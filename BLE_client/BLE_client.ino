/*
Code for the BLE client which will receive data posted by the server (Sleeve chip)

~30 ms latency
*/

#include <41x_ESP_library.h>           // for user made functions and constants
#define DATA_PRINT_REFRESH_TIME 250U  // in ms
#define MOTOR_CONTROL_ERROR_THRESHOLD (float)0.0001  // in full rotations (0.001)

#define HARDNESS_CHANGE_THRESHOLD 10.0F

// the data we recieve from the server each time a notification is triggered
typedef struct {
  ushort max_FSR_reading;
  float estimated_hardness;
} sampling_struct;

// The remote service we wish to connect to.
static const BLEUUID serviceUUID(SERVICE_UUID);
// The characteristic of the remote service we are interested in.
static const BLEUUID charUUID(CHARACTERISTIC_UUID);

// BLE variables
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
static BLEAdvertisedDevice *myDevice = NULL;

// Client constants and counters
bool extract_data(char *string, const char *delim, sampling_struct *struct_to_fill);
ulong last_notify_invoked = 0;
bool recently_invoked_notify = false;

const MOTOR_DIRECTION motor_tighten = CW;  // tightening motor is either CW or CCW. Loosening is always the opposite direction
const MOTOR_DIRECTION motor_loosen = (motor_tighten == CW) ? CCW : CW;

float desired_motor_position_global = 0.0;

const float VIBRATION_MOTOR_PERCENTAGE = (1.0 / NUMBER_OF_VIBRATION_MOTORS);  //  0-1

// Mutex for ensuring safe variable reads and writes of the desired motor position
SemaphoreHandle_t mutex;

// motor task prototype
void motor_task(void *p_params);

// Task handle for the motor task
TaskHandle_t motor_task_handle = NULL;

// a core 0 task that will make sure the motor goes to desired position
void motor_task(void *p_params) {
  float error = 0.0, previous_error = 0.0;
  float cumulative_error = 0.0;
  float current_motor_position = 0.0;                           // in rotations
  float current_motor_speed = 0.0, previous_motor_speed = 0.0;  // in rotations per second
  MOTOR_DIRECTION direction;
  ulong motor_start = 0;

  // PID coefficients
  const float Kp = 1.4, Ki = 0.0, Kd = 0.20;

  for (;;) {
    // aquire lock
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
      error = desired_motor_position_global - current_motor_position;
      //release lock
      xSemaphoreGive(mutex);
    }
    cumulative_error += error;
    float delta_error = error - previous_error;

    float P_term = Kp * error;
    float I_term = Ki * cumulative_error;
    float D_term = Kd * delta_error;

    float motor_speed_factor = P_term + I_term + D_term;
    motor_speed_factor = constrain(motor_speed_factor, -1.0, 1.0);

    if (fabsf(error) < MOTOR_CONTROL_ERROR_THRESHOLD) {
      ulong time_spent = micros() - motor_start;  // account for the distance caused by the motor spinning one last time
      brake_DC_motor();
      delay(5);
      sleep_DC_motor();  // sleeping saves power but won't resist movement
      float distance = distance_traveled_by_motor(previous_motor_speed, time_spent);
      current_motor_position += (direction == motor_tighten) ? distance : -distance;

      vTaskSuspend(NULL);  //suspend the task since we are at the desired point. We will resume when the main task says to (when desired_motor_position changes).
      cumulative_error = 0.0;
      previous_motor_speed = 0.0;
      continue;  // back to start of loop to check the desired motor position (which must have changed if the task is resumed)
    }

    // Determine motor direction and magnitude
    direction = (motor_speed_factor >= 0) ? motor_tighten : motor_loosen;
    float drive_strength = fabsf(motor_speed_factor);  // Use absolute value for PWM strength
    if (previous_motor_speed == 0.0) {
      // give the motor an initial boost when it starts from rest
      drive_strength = 0.75;
    }

    float motor_PWM_value = adjusted_motor_PWM_value(drive_strength, DC_MOTOR);
    motor_PWM_value = constrain(motor_PWM_value, 0, MAX_PWM_VALUE);

    // current_motor_speed = get_DC_motor_speed_from_table(drive_strength, direction);
    current_motor_speed = 0.95 * get_DC_motor_speed(drive_strength);  // convert speed ratio into rotations per second

    // Update position based on motor movement
    ulong time_spent = (motor_start == 0) ? 0 : (micros() - motor_start);
    motor_start = micros();                      // Update motor_start time for next loop
    drive_DC_motor(direction, motor_PWM_value);  // start spinning the motor at desired speed

    if (time_spent != 0) {
      update_motor_position(current_motor_position, direction, previous_motor_speed, time_spent, motor_tighten);
    }

    previous_error = error;
    previous_motor_speed = current_motor_speed;
  }
}

// what to do upon recieving BLE notification (ie a characteristic was updated from the server)
static void notifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic, uint8_t *pData, size_t length, bool isNotify) {
  last_notify_invoked = millis();
  recently_invoked_notify = true;
  static sampling_struct data = {0};
  // static float current_hardness, previous_hardness = 0.0;

  // extract data into struct
  if (!extract_data((char *)pData, " ", &data)) {
    Serial.printf("Could not extract data succesfully. Aborting\n");
    handle_error();
  }

  /**************************************************************************
  ***************************************************************************

  Set the motor position to the FSR readings on the other core
  
  ***************************************************************************
  **************************************************************************/

  float ratio_DC_motor = map_ratio(data.max_FSR_reading, ABS_FSR_THRESHOLD, MAX_ADC_VALUE);
  // use mutex here since motor task reads the value
  if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
    desired_motor_position_global = ratio_DC_motor * (MAX_MOTOR_ROTATIONS);  // the number of ROTATIONS we want to get to
    xSemaphoreGive(mutex);
  }

  // if task is suspended (done going to set point) then resume it. If not, it will read the new global set point
  if (eTaskGetState(motor_task_handle) == eSuspended) {
    // vTaskResume() is safe to call on unsuspended tasks but better to check
    vTaskResume(motor_task_handle);  // now that the global variable has been updated, resume the task
  }

  /**************************************************************************
  ***************************************************************************

  Set vibration motors to indicate hardness of object 

  ***************************************************************************
  **************************************************************************/

  float ratio_hardness = map_ratiof(data.estimated_hardness, 0, MAX_HARDNESS_VALUE);

  float strength[NUMBER_OF_VIBRATION_MOTORS] = {0.0};  // only for printing

  uchar number_of_motors_fully_on = (uchar)(ratio_hardness / VIBRATION_MOTOR_PERCENTAGE);  // want integer division here (the number of motors that can be FULLY on)

  for (uchar i = 0; i < number_of_motors_fully_on; i++) {
    analogWrite(VIBRATION_PWM_PINS[i], adjusted_motor_PWM_value(1.0, VIBRATION_MOTOR));  // fully turn on
    strength[i] = 100.0;
  }
  // last pin gets the remainder
  float remaining_percentage = (ratio_hardness - (number_of_motors_fully_on * VIBRATION_MOTOR_PERCENTAGE));  // also between 0 and 1
  float last_vibration_motor_ratio = constrain((remaining_percentage / VIBRATION_MOTOR_PERCENTAGE), 0.0, 1.0); // ensure ratio is within bounds
  ushort last_vibration_motor_PWM = (ushort)(adjusted_motor_PWM_value(last_vibration_motor_ratio, VIBRATION_MOTOR));

  analogWrite(VIBRATION_PWM_PINS[number_of_motors_fully_on], last_vibration_motor_PWM);
  strength[number_of_motors_fully_on] = ((100.0 * (last_vibration_motor_PWM - adjusted_motor_PWM_value(0.0001, VIBRATION_MOTOR))) / effective_PWM_range(VIBRATION_MOTOR));

  // occasionally print data received (for debugging)
  static uint last = DATA_PRINT_REFRESH_TIME;
  if (millis() - last >= DATA_PRINT_REFRESH_TIME) {
    last = millis();
    Serial.printf("\ndata transmitted: %04hu (max FSR) | Force strength: %.2f%% | %+08.3f\n", data.max_FSR_reading, (ratio_DC_motor * 100.0), data.estimated_hardness);
    Serial.printf("Vibration motors:");
    for (ushort i = 0; i < NUMBER_OF_VIBRATION_MOTORS; i++) {
      if (!i) {
        Serial.printf(" %04.2f%%", strength[i]);
      } else {
        Serial.printf(" | %04.2f%%", strength[i]);
      }
    }
    Serial.printf("\tHardness value: %7.3f\n", data.estimated_hardness);
  }
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient *pclient) {
    Serial.printf("Established connection\n");
  }

  void onDisconnect(BLEClient *pclient) {
    connected = false;
    analogWrite(LED_BUILTIN, 0);  // turn off blue LED
    Serial.printf("Disconnected\n");
  }
};

bool connectToServer() {
  if (myDevice == NULL) {
    // in case myDevice is NULL
    return false;
  }
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  pClient->setMTU(MAX_BLE_MESSAGE_LENGTH);  //set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(BLEUUID(serviceUUID).toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    // create non const copies or instead use
    // Serial.println(charUUID.toString().c_str());
    Serial.println(BLEUUID(charUUID).toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    String value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(notifyCallback);
  }

  connected = true;
  analogWrite(LED_BUILTIN, LED_BUILTIN_BRIGHTNESS);  // turn on LED to indicate connection
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());
    // Serial.printf("has service: %d\n", (int)advertisedDevice.haveServiceUUID());
    // Serial.printf("has service UUID: %s\n", BLEUUID(advertisedDevice.getServiceUUID()).toString().c_str());
    // Serial.printf("looking for UUID: %s\n", BLEUUID(serviceUUID).toString().c_str());
    // Serial.printf("Match status: %s\n", advertisedDevice.isAdvertisingService(serviceUUID) ? "found" : "not found");

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {
      Serial.printf("Found our device!\n");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    }  // Found our server
  }    // onResult
};     // MyAdvertisedDeviceCallbacks

void setup() {
  Serial.begin(BAUD_RATE_ESP32);
  while (!Serial);
  if (!setup_motor_pins() || !(setup_vibration_pins())) {
    Serial.printf("Error setting up motor or vibration pins\n");
    handle_error();
  }
  sleep_DC_motor(); // ensure motor is off
  analogReadResolution(ADC_RESOLUTION_BITS);  // Sets the ADC resolution to library constamt
  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init(""); // client does not need a name

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for a few seconds.
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(3, false);

  
  setup_indication_LED();

  // RGB LED setup (board identification)
  setup_RGB_LED(CLIENT_HEX_RGB_COLOUR);

  // set up our mutex for the desired_motor_position_global variable
  mutex = xSemaphoreCreateMutex();

  /*
  start the motor task on the other core. It will never finish, but will be suspended
  when not driving the motor towards the set point to save system resources
  */
  xTaskCreatePinnedToCore(motor_task,
                          "motor task",
                          1024 * 2.5,                           // stack size in bytes
                          (void *)NULL,                       // parameters
                          0,                                  // task priority
                          &motor_task_handle,                 // task handle
                          (TASK_RUNNING_CORE == 0) ? 1 : 0);  // whatever core the loop is using, use the other one (loop should be on core 1 by default)
  Serial.printf("Finished setup\n");
}  // End of setup.

void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
    } else {
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, reset the motor to position 0 
  // once we have not recieved data for a small period of time
  if (connected) {
    if (recently_invoked_notify && (millis() - last_notify_invoked >= 75)) {
      // turn everything off
      if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        desired_motor_position_global = 0.0;
        xSemaphoreGive(mutex);
      }

      // motor will likely not be in position 0 so resume it to go to 0
      if (eTaskGetState(motor_task_handle) == eSuspended) {
        vTaskResume(motor_task_handle);
      }

      for (ushort i = 0; i < NUMBER_OF_VIBRATION_MOTORS; i++) {
        analogWrite(VIBRATION_PWM_PINS[i], 0);
      }

      recently_invoked_notify = false;
    }
  } else if (doScan) {
    Serial.printf("Attempting to scan for server...\n");
    BLEScan *pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setInterval(1349);
    pBLEScan->setWindow(449);
    pBLEScan->setActiveScan(true);
    pBLEScan->start(5, false);
  }
}  // End of loop

// requires the arguments that it takes to be ordered properly to match data string
bool extract_data(char *string, const char *delim, sampling_struct *struct_to_fill) {
  if (!string || !struct_to_fill) {
    Serial.printf("Error: NULL pointer in extract_data parameters\n");
    return false;
  }

  char *token = strtok(string, delim);
  if (token == NULL) {
    Serial.printf("Error: First token missing in input\n");
    return false;
  }
  struct_to_fill->max_FSR_reading = (ushort)atoi(token);

  token = strtok(NULL, delim);
  if (token == NULL) {
    Serial.printf("Error: Second token missing in input\n");
    return false;
  }
  struct_to_fill->estimated_hardness = (float)atof(token);
  return true;
}
