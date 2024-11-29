/*
Code for the BLE server (Glove chip) which will send data to the client ESP32 (Sleeve chip) when the client should activate motors
~30 ms latency
*/

#include <41x_ESP_library.h> // for user made functions and constants
#include "BLEDevice.h" // basic BLE functions and classes
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <deque>

#define min(a, b) ((a) < (b) ? (a) : (b))

#define DERIVATIVE_PERIOD_FSR_MILLISECONDS 1U //ms
#define DERIVATIVE_PERIOD_FLEX_SENSOR_MILLISECONDS 1U //ms

// Print whenever the derivatives are computed
// #define SENSOR_PRINT_REFRESH_TIME min(DERIVATIVE_PERIOD_FSR_MILLISECONDS, DERIVATIVE_PERIOD_FLEX_SENSOR_MILLISECONDS)
#define SENSOR_PRINT_REFRESH_TIME 100U

void start_hosting_server(void);
float pounds_to_newtons(float pounds);
float newtons_to_pounds(float newtons);
ushort max(ushort *array_of_readings, uchar number_of_elements);
ushort average_reading(const byte pin_to_read, const uint number_of_readings);
float maximum_detectable_FSR_force_in_newtons(const uint resistor_value_ohms);
float update_sensor_window(const short new_value, std::deque<short>& window);
float update_hardness_window(const float new_value, std::deque<float>& window);
float get_hardness_estimate(const ushort FSR_reading, float flex_derivative);

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

#define FLEX_WINDOW_SIZE 50U
#define HARDNESS_WINDOW_SIZE 25U

std::deque<short> flex_window = {0};  // Rolling window for derivatives

std::deque<float> hardness_window = {0};  // Rolling window for hardness
std::deque<float> flex_derivative_window = {0};  // Rolling window for flex derivatives

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    analogWrite(LED_BUILTIN, LED_BUILTIN_BRIGHTNESS); // turn on LED to indicate connection
    Serial.printf("A device has connected to the server\n");
  };

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    analogWrite(LED_BUILTIN, 0); // turn off LED
    Serial.printf("A device has disconnected from the server\n");
  }
};

void setup() {
  Serial.begin(BAUD_RATE_ESP32);
  while(!Serial);
  analogReadResolution(ADC_RESOLUTION_BITS);  // Sets the ADC resolution to desired number of bits (normally 12)
  
  setup_indication_LED();
  analogWrite(LED_BUILTIN, 0); // turn off LED

  // RGB LED setup (board identification)
  setup_RGB_LED(SERVER_HEX_RGB_COLOUR);

  if (!verify_FSR_pins()) {
    Serial.printf("Could not setup FSR pins\n");
    handle_error();
  }

  start_hosting_server();
  Serial.println("BLE server is now advertising...");
}

void loop() {
  if (deviceConnected) {
    static char message[MAX_BLE_MESSAGE_LENGTH + 1] = "";
    static ulong last_all_sensors = SENSOR_PRINT_REFRESH_TIME;
    ushort FSR_readings[NUMBER_OF_FSRS];
    float max_FSR_reading, flex_reading;
    static float previous_FSR_reading = 0, previous_flex_reading = 0;
    static float flex_derivative = 0.0, FSR_derivative = 0.0;
    float time_elapsed_1 = DERIVATIVE_PERIOD_FSR_MILLISECONDS;
    float time_elapsed_2 = DERIVATIVE_PERIOD_FLEX_SENSOR_MILLISECONDS;
    // static ulong previous_reading_time = 0, reading_time = 0;
    float hardness = 0.0;
    
    static ulong last_FSR_derivative_time = 0, last_flex_derivative_time = 0;
    // static ulong last_derivative_time = 0;
    
    for (int i = 0; i < NUMBER_OF_FSRS; i++) {
      FSR_readings[i] = analogRead(FSR_ANALOG_READING_PINS[i]);
    }

    // gather a number of readings and remove outliers
    flex_reading = update_sensor_window(analogRead(FLEX_SENSOR_ANALOG_READING_PIN), flex_window);

    // reading_time = micros();
    max_FSR_reading = max(FSR_readings, NUMBER_OF_FSRS);
    // max_FSR_reading = update_sensor_window((short) max_FSR_reading, FSR_window);

    float delta_FSR = (max_FSR_reading - previous_FSR_reading);
    float delta_flex = (flex_reading - previous_flex_reading);

    time_elapsed_1 = millis() - last_FSR_derivative_time;
    if (time_elapsed_1 > DERIVATIVE_PERIOD_FSR_MILLISECONDS) {
      last_FSR_derivative_time = millis();
      // take derivative readings and post them
      // if (previous_reading_time == 0) {
      //   FSR_derivative = 0.0; // not differentiable when there is no previous value or time passed
      //   flex_derivative = 0.0;
      // } else {
        // time_elapsed = (float) ((reading_time - previous_reading_time));
        time_elapsed_1 = 1.0;
        FSR_derivative = (delta_FSR / time_elapsed_1) * 1.0; // may need to scale derivative (?)
        previous_FSR_reading = max_FSR_reading;
        // previous_reading_time = reading_time;
      // }
    }
    if ((time_elapsed_2 = (millis() - last_flex_derivative_time) > DERIVATIVE_PERIOD_FLEX_SENSOR_MILLISECONDS)) {
      last_flex_derivative_time = millis();
      time_elapsed_2 = 1.0;
      flex_derivative = (delta_flex / time_elapsed_2) * 1.0;
      flex_derivative = update_hardness_window(flex_derivative, flex_derivative_window);
      // flex_derivative = (fabsf(flex_derivative) >= 0.3) ? flex_derivative : 0;
      previous_flex_reading = flex_reading;
    }
    bool sending = false;
    if ((max_FSR_reading > ABS_FSR_THRESHOLD) && (flex_reading > ABS_FLEX_SENSOR_THRESHOLD)) {
      sending = true;
      hardness = get_hardness_estimate(max_FSR_reading, flex_derivative);
      snprintf(message, MAX_BLE_MESSAGE_LENGTH, "%04hu %07.3f", (ushort)max_FSR_reading, hardness);
      
      pCharacteristic->setValue(message); // Update the characteristic value
      pCharacteristic->notify(); // Notify connected client about the updated value
      delay(2); // small delay to not overflow BLE stack (too many notifications crashes the client)
    } else {
      hardness = 0.0;
    }
    
    // do not spam serial monitor
    if (millis() - last_all_sensors >= SENSOR_PRINT_REFRESH_TIME) {
      last_all_sensors = millis();
      for (uchar i = 0; i < NUMBER_OF_FSRS; i++) {
        Serial.printf("FSR %d reading: %04hu | ", i + 1, FSR_readings[i]);
      }
      Serial.printf("Flex sensor reading: %08.2f ", flex_reading);
      // Serial.printf("\tFSR derivative: %+09.3f", FSR_derivative);
      Serial.printf("\tFlex derivative: %+09.3f", flex_derivative);
      Serial.printf("\tPercieved Hardness: %+09.3f\t%s\n", hardness, (sending) ? "DATA SENT" : "NO DATA SENT");
    }
    // end of conditional block
  }

  // Handle disconnection and reconnection
  if (!deviceConnected && oldDeviceConnected) {
    delay(500); // Give the Bluetooth stack the chance to get things ready
    pServer->startAdvertising();  // Restart advertising
    static ulong last = 5000;
    if (millis() - last >= 5000) {
      Serial.println("Nothing connected to server right now. Advertising service again.");
      last = millis();
    }
    oldDeviceConnected = deviceConnected;
  }
  
  if (deviceConnected && !oldDeviceConnected) {
    // Handle device connection event here
    oldDeviceConnected = deviceConnected;
  }
}

void start_hosting_server(void) {
  BLEDevice::init("ESP32_Server");

  // Create BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  
  // Create BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      // BLECharacteristic::PROPERTY_READ |
                      // BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  BLEDescriptor * pBLE2901 = new BLEDescriptor((uint32_t)2901);
  pCharacteristic->addDescriptor(pBLE2901);

  /*
  We need a client characteristic configuration for notify()
  */
  BLE2902 * pBLE2902 = new BLE2902();
  pBLE2902->setNotifications(true);
  pCharacteristic->addDescriptor(pBLE2902); // new BLE2902() will also enable notifications

  // pCharacteristic->setValue("Hello from ESP32 Server!");
  // start service after all characteristics added
  pService->start();

  // Start advertising once we have characteristics 
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  return;
}

float pounds_to_newtons(float pounds) {
  return pounds >= 0 ? pounds * 4.44822 : 0;  // 1 pound ≈ 4.44822 newtons
}

float newtons_to_pounds(float newtons) {
  return newtons >= 0 ? newtons / 4.44822 : 0;  // 1 newton ≈ 0.224809 pounds
}

ushort max(ushort *array_of_readings, uchar number_of_elements) {
  if (!array_of_readings || number_of_elements == 0) { // Check for null or empty array
    if (!array_of_readings) {
      null_handler(PASSED_NULL_PTR, (const char *)"max");
    }
    if (Serial) Serial.printf("Number of elements provided was 0\n");
    return 0;
  }

  ushort max_val = *array_of_readings; // Initialize max with the first element
  for (uchar i = 1; i < number_of_elements; i++) { // Start loop from 1
    if (array_of_readings[i] > max_val) {
      max_val = array_of_readings[i];
    }
  }
  return max_val;
}

ushort average_reading(const byte pin_to_read, const uint number_of_readings) {
  ulonglong sum = 0;
  for (uint i = 0; i < number_of_readings; i++) {
    sum += analogRead(pin_to_read);
  }
  return (ushort)(sum / number_of_readings);
}

// Function to calculate Fmax based on the resistor value
float maximum_detectable_FSR_force_in_newtons(const uint resistor_value_ohms) {
  // Use points (1000, 104.64), (10,000, 10.35), (100,000, 1.44) and linear interpolation to estimate the function

  // coordinate coords[3] = { {1000, 104.64}, {10000, 10.35}, {100000, 1.44} };
  // return pow(10, 4.787004002059344) * pow(resistor_value_ohms, -0.93066762);

  switch(resistor_value_ohms) {
    case 1000:
      return 104.64;
    case 10000:
      return 10.35;
    case 100000:
      return 1.44;
    default:
      Serial.println("Resistance must be 1K, 10K, or 100K");
      return -10000;
  }
}

float update_sensor_window(const short new_value, std::deque<short>& window) {
  if (window.size() >= FLEX_WINDOW_SIZE) window.pop_front();
  window.push_back(new_value);

  // Filter using IQR
  std::vector<short> sorted_window(window.begin(), window.end());
  std::sort(sorted_window.begin(), sorted_window.end());

  short Q1 = sorted_window[FLEX_WINDOW_SIZE / 4];
  short Q3 = sorted_window[(3 * FLEX_WINDOW_SIZE) / 4];
  short IQR = Q3 - Q1;

  float mean = 0.0;
  int count = 0;
  for (auto value : sorted_window) {
    if (value >= (Q1 + 0.0 * IQR) && value <= (Q3 - 0.0 * IQR)) {
      mean += value;
      count++;
    }
  }

  mean /= count;
  if (count == 0) {
    Serial.printf("No readings in band!\n");
    return (new_value + sorted_window[IQR / 2]) / 2.0; // Fall back to the new value no value is close enough 
  }
  return mean;
}

float update_hardness_window(const float new_value, std::deque<float>& window) {
  if (window.size() >= HARDNESS_WINDOW_SIZE) window.pop_front();
  window.push_back(new_value);

  // Filter using IQR
  std::vector<float> sorted_window(window.begin(), window.end());
  std::sort(sorted_window.begin(), sorted_window.end());

  float Q1 = sorted_window[HARDNESS_WINDOW_SIZE / 4];
  float Q3 = sorted_window[(3 * HARDNESS_WINDOW_SIZE) / 4];
  float IQR = Q3 - Q1;

  float mean = 0.0;
  int count = 0;
  for (auto value : sorted_window) {
    if (value >= (Q1 - 0.25 * IQR) && value <= (Q3 + 0.25 * IQR)) {
      mean += value;
      count++;
    }
  }

  mean /= count;
  if (count == 0) {
    Serial.printf("Could not find mean for hardness / flex derivative\n");
    return (new_value + sorted_window[IQR / 2]) / 2.0; // Fall back to the new value no value is close enough 
  }
  return mean;
}

float get_hardness_estimate(const ushort FSR_reading, float flex_derivative) {
  
  const float EPSILON = 0.01;
  const float MAX_FLEX_DERIVATIVE = 8.5;
  static float previous_hardness = 0.0;

  if (flex_derivative <= 0.0) {
    if (flex_derivative == 0.0) flex_derivative = EPSILON; // Avoid division by zero

    // hardness should decrease with negative flex derivative but not by much (flex derivative is noisy)
    // if a negative flex sensor derivative is sustained hardness should go to 0
    return update_hardness_window(previous_hardness * 0.95, hardness_window);
  }

  // normalize FSR data
  float scaled_FSR_reading = (((float)(FSR_reading - ABS_FSR_THRESHOLD)) / (MAX_ADC_VALUE - ABS_FSR_THRESHOLD));
  scaled_FSR_reading = constrain(scaled_FSR_reading, 0.0, 1.0);

  // normalize flex data
  // float scaled_flex_derivative = flex_derivative / (MAX_FLEX_SENSOR_VALUE - ABS_FLEX_SENSOR_THRESHOLD);
  float scaled_flex_derivative = flex_derivative / MAX_FLEX_DERIVATIVE; // flex derivative tends not to go above ~8.0
  scaled_flex_derivative = constrain(scaled_flex_derivative, EPSILON, 1.0);

  float scaling_factor = 1.1;
  float hardness_estimate = (scaling_factor * scaled_FSR_reading) / (scaled_flex_derivative);

  hardness_estimate = constrain(hardness_estimate, 0.0, MAX_HARDNESS_VALUE);
  hardness_estimate = update_hardness_window(hardness_estimate, hardness_window);
  previous_hardness = hardness_estimate;
  return hardness_estimate;
}
