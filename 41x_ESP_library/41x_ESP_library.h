/*
Library for use in the 41x Final Design Project.
Consists of constants and functions designed for use with ESP32-WROOM-32E.

One ESP32 serves as the client and sends data in small chunks to the server
ESP32.
*/

// Include libraries used in both client and server sketches
#include "BLEDevice.h" // BLE functionality for communication
#include "Freenove_WS2812_Lib_for_ESP32.h" // Library for RGB LED control

/*****************************************************************************
******************************************************************************
                                Typedefs
******************************************************************************
*****************************************************************************/

// Enum for extracting RGB components from hex codes
typedef enum {
  RED,
  GREEN,
  BLUE
} RGB;

// Enum for null pointer/memory allocation error codes
typedef enum {
  PASSED_NULL_PTR,
  FAILED_MALLOC
} NULL_ERROR;

// Shorthand typedefs for common unsigned data types
typedef unsigned char uchar;
typedef unsigned int uint;
typedef unsigned long ulong;
typedef unsigned long long ulonglong;
typedef unsigned short ushort;

/*****************************************************************************
******************************************************************************
                              Threshold Constants
******************************************************************************
*****************************************************************************/

// Thresholds for sensor data interpretation
#define ABS_FSR_THRESHOLD 350U

#define ABS_FLEX_SENSOR_THRESHOLD 200U
#define MAX_FLEX_SENSOR_VALUE 340U

#define MAX_HARDNESS_VALUE 100.0F

/*****************************************************************************
******************************************************************************
                         ESP32 Configuration Constants
******************************************************************************
*****************************************************************************/

// General ESP32 configuration
#define BAUD_RATE_ESP32 115200U
#define ADC_RESOLUTION_BITS (byte)12 // ADC resolution in bits
#define SENSOR_INPUT_VOLTAGE 3.3F    // Input voltage for sensors

// Calculate maximum ADC value
ulonglong int_power(const int base, uint exp);
const ushort MAX_ADC_VALUE = int_power(2, ADC_RESOLUTION_BITS) - 1;

/*****************************************************************************
******************************************************************************
                           Function Prototypes
******************************************************************************
*****************************************************************************/

void setup_indication_LED(void);
byte extract_RGB_colour_from_hex(const uint RGB_code, const RGB desired_colour);
void setup_RGB_LED(uint RGB_desired_colour);
bool is_ADC_capable(const byte pin);
bool is_DAC_capable(const byte pin);
bool is_PWM_capable(const byte pin);
bool is_strapping_pin(const byte pin);
void null_handler(NULL_ERROR error_code, const char *function_name);
ushort adjusted_PWM_value(const float desired_strength, const float max_voltage,
                          const float min_voltage, const ushort max_PWM_value);
void handle_error(void);
inline float map_ratio(const ushort reading, const ushort threshold, const ushort maxValue);
inline float map_ratiof(const float value_to_map, const float threshold, const float maxValue);

// Array of GPIO pins with specific capabilities
const byte ADC_CAPABLE_GPIO_PINS[] = {39, 36, 34, 35, 32, 33, 25, 26,
                                      27, 14, 12, 13, 4,  0,  2,  15};
const byte DAC_CAPABLE_GPIO_PINS[] = {25, 26};
const byte PWM_CAPABLE_GPIO_PINS[] = {32, 33, 25, 26, 27, 14, 12, 13,
                                      23, 22, 1,  3,  21, 19, 18, 5,
                                      17, 16, 4,  0,  2,  15};
const byte STRAPPING_GPIO_PINS[] = {12, 5, 0, 2, 15};

/*****************************************************************************
******************************************************************************
                         Server (Glove ESP) Constants
******************************************************************************
*****************************************************************************/

#define NUMBER_OF_FSRS (byte)3 // Number of force-sensitive resistors

// GPIO pins for FSR readings (must be ADC capable according to board pinout)
const byte FSR_ANALOG_READING_PINS[NUMBER_OF_FSRS] = {36, 39, 34};
#define FLEX_SENSOR_ANALOG_READING_PIN (byte)32 // GPIO pin for flex sensor

// Function to verify that assigned FSR pins are ADC capable
bool verify_FSR_pins();

/*****************************************************************************
******************************************************************************
                         Client (Sleeve ESP) Constants
******************************************************************************
*****************************************************************************/

// DC Motor Configuration
#define DC_MOTOR_INPUT_VOLTAGE 18.0F // Voltage source for DC motor
#define MAX_MOTOR_ROTATIONS 5.0F // Maximum rotations for desired force output
#define DC_MOTOR_MINIMUM_VOLTAGE 10.1F // Minimum voltage to start motor (10.7 previously)
#define MOTOR_DRIVER_PWM 10000U // PWM frequency (1K - 100K Hz per datasheet)

// Motor direction types
typedef enum {
  CW, // Clockwise
  CCW // Counterclockwise
} MOTOR_DIRECTION;

// Motor types
typedef enum {
  VIBRATION_MOTOR,
  DC_MOTOR
} MOTOR_TYPE;

// Vibration Motor Configuration
#define VIBRATION_MOTOR_INPUT_VOLTAGE 9.0F
#define NUMBER_OF_VIBRATION_MOTORS (byte)3
#define VIBRATION_MOTOR_MINIMUM_INPUT_VOLTAGE 2.5F

// Force and Flex Sensor Ranges
#define RANGE_OF_FORCE_VALUES (MAX_ADC_VALUE - ABS_FSR_THRESHOLD)
#define RANGE_OF_FLEX_VALUES (MAX_FLEX_SENSOR_VALUE - ABS_FLEX_SENSOR_THRESHOLD)

// GPIO pins for motor operation
const byte VIBRATION_PWM_PINS[NUMBER_OF_VIBRATION_MOTORS] = {19, 18, 5};
const byte DC_MOTOR_DIRECTION_PINS[2] = {23, 22}; // Direction pins: index corresponds to MOTOR_DIRECTION enum

// Motor utility functions
ushort effective_PWM_range(MOTOR_TYPE type);
ushort adjusted_motor_PWM_value(const float desired_strength,
                                const MOTOR_TYPE type);
bool drive_DC_motor(MOTOR_DIRECTION direction, const ushort PWM_strength);
bool setup_vibration_pins();
bool setup_motor_pins();
float get_DC_motor_speed(const float power_ratio);
inline void sleep_DC_motor();
inline void brake_DC_motor();
float get_motor_speed_factor(const float error, const float previous_speed);
float get_DC_motor_speed_from_table(float speed_ratio,
                                    MOTOR_DIRECTION direction_of_spin);
inline void swap_direction(MOTOR_DIRECTION &direction);
inline void
update_motor_position(float &current_position, MOTOR_DIRECTION direction,
                    const float DC_motor_speed_rotations_per_second,
                    const ulong time_spent_microseconds,
                    const MOTOR_DIRECTION motor_tightening_direction);

/*****************************************************************************
******************************************************************************
                         LED Configuration Constants
******************************************************************************
*****************************************************************************/

// RGB LED configuration
#define LEDS_COUNT 1U
#define LEDS_PIN 16U
#define LED_CHANNEL 15U
#define STRIP_BRIGHTNESS 35U // Maximum brightness: 255

// WS2812 RGB LED object
Freenove_ESP32_WS2812 strip = Freenove_ESP32_WS2812(LEDS_COUNT, LEDS_PIN, LED_CHANNEL, TYPE_GRB);

// Built-in LED
#define LED_BUILTIN (byte)2 // Blue LED indicates connection status

/*****************************************************************************
******************************************************************************
                         PWM Configuration Constants
******************************************************************************
*****************************************************************************/

// PWM general settings
#define PWM_FREQUENCY 1000U       // PWM frequency in Hz
#define PWM_RESOLUTION 8U         // Resolution in bits (0-255)
#define LED_BUILTIN_BRIGHTNESS 5U // Brightness level for built-in LED

// Calculate maximum PWM value
const ushort MAX_PWM_VALUE = int_power(2, PWM_RESOLUTION) - 1;

// Default LED colors for server and client
#define SERVER_HEX_RGB_COLOUR 0xFC6603 // Yellowish
#define CLIENT_HEX_RGB_COLOUR 0x00FF88 // Cyan-green

/*****************************************************************************
******************************************************************************
                          BLE Configuration Constants
******************************************************************************
*****************************************************************************/

// BLE service and characteristic UUIDs
#define SERVICE_UUID "2c4487c8-117a-4e2a-9fcd-65bb62268f58"
#define CHARACTERISTIC_UUID "255d99de-80af-4411-a250-872be96ad0da"

// BLE message length limitations
#define MAX_BLE_MESSAGE_LENGTH 35U // Default max is 20 bytes; add 3 for BLE headers

// Multithreading configuration for FreeRTOS
#if CONFIG_FREERTOS_UNICORE
#define TASK_RUNNING_CORE 0
#else
#define TASK_RUNNING_CORE 1
#endif

/*****************************************************************************
******************************************************************************
                         Function Definitions
******************************************************************************
*****************************************************************************/

// Extract a specific color (RED, GREEN, BLUE) from an RGB hex code
byte extract_RGB_colour_from_hex(unsigned int RGB_code, RGB desired_colour) {
  switch (desired_colour) {
  case RED:
    return (byte)((RGB_code & 0xFF0000) >> 16); // Extract red
  case GREEN:
    return (byte)((RGB_code & 0x00FF00) >> 8); // Extract green
  case BLUE:
    return (byte)(RGB_code & 0x0000FF); // Extract blue
  default:
    if (Serial) {
      Serial.printf("Error: Invalid color requested\n");
    }
    return 0;
  }
}

// Set up the indication LED (blue built-in)
void setup_indication_LED(void) {
  ledcAttach(LED_BUILTIN, PWM_FREQUENCY, PWM_RESOLUTION);
}

// Initialize RGB LED with a specified color
void setup_RGB_LED(uint RGB_desired_colour) {
  strip.begin();
  strip.setBrightness(STRIP_BRIGHTNESS);
  strip.setLedColorData(0, extract_RGB_colour_from_hex(RGB_desired_colour, RED),
                        extract_RGB_colour_from_hex(RGB_desired_colour, GREEN),
                        extract_RGB_colour_from_hex(RGB_desired_colour, BLUE));
  strip.show();
}

// Raise an integer to a power
ulonglong int_power(const int base, uint exponent) {
  ulonglong result = 1;
  while (exponent > 0) {
    result *= base;
    exponent--;
  }
  return result;
}

// Check if a pin is ADC capable
bool is_ADC_capable(const byte pin) {
  for (byte ADC_pin : ADC_CAPABLE_GPIO_PINS) {
    if (ADC_pin == pin) {
      return true;
    }
  }
  return false;
}

// Check if a pin is DAC capable
bool is_DAC_capable(const byte pin) {
  for (byte DAC_pin : DAC_CAPABLE_GPIO_PINS) {
    if (DAC_pin == pin) {
      return true;
    }
  }
  return false;
}

// Check if a pin is PWM capable
bool is_PWM_capable(const byte pin) {
  for (byte PWM_pin : PWM_CAPABLE_GPIO_PINS) {
    if (PWM_pin == pin) {
      return true;
    }
  }
  return false;
}

// Check if a pin is a strapping pin
bool is_strapping_pin(const byte pin) {
  for (byte strapping_pin : STRAPPING_GPIO_PINS) {
    if (strapping_pin == pin) {
      return true;
    }
  }
  return false;
}

// Handle null pointer or malloc errors
void null_handler(NULL_ERROR error_code, const char *function_name = "") {
  const char *cause = (error_code == PASSED_NULL_PTR) ? "NULL pointer passed" : "Malloc failed";

  if (Serial) {
    if (function_name[0] != '\0') {
      Serial.printf("Error: %s in function %s\n", cause, function_name);
    } else {
      Serial.printf("Error: %s\n", cause);
    }
  }

  // Halt program
  handle_error();
}

// Verify if all FSR pins are ADC capable
bool verify_FSR_pins() {
  for (uchar i = 0; i < NUMBER_OF_FSRS; i++) {
    if (!is_ADC_capable(FSR_ANALOG_READING_PINS[i])) {
      if (Serial) {
        Serial.printf("Error: Pin %d for FSR is not ADC capable\n", (int)FSR_ANALOG_READING_PINS[i]);
      }
      return false;
    }
  }
  return true;
}

// Drive the DC motor with a specific PWM strength and direction
bool drive_DC_motor(MOTOR_DIRECTION direction, const ushort PWM_strength) {
  if (PWM_strength > MAX_PWM_VALUE) {
    if (Serial) {
      Serial.printf("Invalid PWM strength: %hu (Max: %hu)\n", PWM_strength, MAX_PWM_VALUE);
    }
    return false;
  }

  switch (direction) {
  case CW:
    analogWrite(DC_MOTOR_DIRECTION_PINS[CW], PWM_strength);
    analogWrite(DC_MOTOR_DIRECTION_PINS[CCW], 0);
    break;
  case CCW:
    analogWrite(DC_MOTOR_DIRECTION_PINS[CW], 0);
    analogWrite(DC_MOTOR_DIRECTION_PINS[CCW], PWM_strength);
    break;
  default:
    if (Serial) {
      Serial.printf("Error: Invalid motor direction\n");
    }
    return false;
  }
  return true;
}

// Configure the pins for the motor
bool setup_motor_pins() {
  // Ensure both pins are PWM capable
  if (!is_PWM_capable(DC_MOTOR_DIRECTION_PINS[CW]) ||
      !is_PWM_capable(DC_MOTOR_DIRECTION_PINS[CCW])) {
    if (Serial) {
      Serial.printf("One or both motor pins are not PWM capable\n");
    }
    return false;
  }

  // Check if pins are strapping pins
  if (is_strapping_pin(DC_MOTOR_DIRECTION_PINS[CW]) ||
      is_strapping_pin(DC_MOTOR_DIRECTION_PINS[CCW])) {
    if (Serial) {
      Serial.printf(
          "Error: Strapping pins should not be used for motor control\n");
    }
    return false;
  }

  // Attach PWM to motor pins
  ledcAttach(DC_MOTOR_DIRECTION_PINS[CW], MOTOR_DRIVER_PWM, PWM_RESOLUTION);
  ledcAttach(DC_MOTOR_DIRECTION_PINS[CCW], MOTOR_DRIVER_PWM, PWM_RESOLUTION);
  return true;
}

// Configure the pins for vibration motors
bool setup_vibration_pins() {
  for (uchar i = 0; i < NUMBER_OF_VIBRATION_MOTORS; i++) {
    if (!is_PWM_capable(VIBRATION_PWM_PINS[i])) {
      if (Serial) {
        Serial.printf("Error: Pin %d is not PWM capable\n", (int)VIBRATION_PWM_PINS[i]);
      }
      return false;
    }
    ledcAttach(VIBRATION_PWM_PINS[i], PWM_FREQUENCY, PWM_RESOLUTION);
  }
  return true;
}

// Generic error handler that halts execution
void handle_error(void) {
  while (1) {
    delay(1000); // Prevent CPU overload
  }
}

// Calculate adjusted PWM value based on input strength and voltage thresholds
ushort adjusted_PWM_value(const float desired_strength, const float max_voltage,
                          const float min_voltage, const ushort max_PWM_value) {
  if (desired_strength < 0.0 || desired_strength > 1.0) {
    if (Serial) {
      Serial.printf("Error: Invalid PWM strength %.6f (must be 0.0 to 1.0)\n",
                    desired_strength);
    }
    handle_error();
  }

  if (desired_strength == 0.0) {
    return 0; // Zero volts is always off
  }

  // Calculate the minimum PWM fraction for the minimum voltage threshold
  float minimum_PWM_fraction = min_voltage / max_voltage;

  // Map desired strength to the effective PWM range
  ushort effective_range =
      max_PWM_value - (ushort)(minimum_PWM_fraction * max_PWM_value);
  return (ushort)(desired_strength * effective_range) +
         (ushort)(minimum_PWM_fraction * max_PWM_value);
}

// Adjust motor PWM value based on type and input strength
ushort adjusted_motor_PWM_value(const float desired_strength, MOTOR_TYPE type) {
  switch (type) {
  case DC_MOTOR:
    return adjusted_PWM_value(desired_strength, DC_MOTOR_INPUT_VOLTAGE,
                              DC_MOTOR_MINIMUM_VOLTAGE, MAX_PWM_VALUE);
  case VIBRATION_MOTOR:
    return adjusted_PWM_value(desired_strength, VIBRATION_MOTOR_INPUT_VOLTAGE,
                              VIBRATION_MOTOR_MINIMUM_INPUT_VOLTAGE,
                              MAX_PWM_VALUE);
  default:
    return 0;
  }
}

// Calculate the effective PWM range for a given motor type
ushort effective_PWM_range(MOTOR_TYPE type) {
  switch (type) {
  case DC_MOTOR:
    return MAX_PWM_VALUE - (DC_MOTOR_MINIMUM_VOLTAGE / DC_MOTOR_INPUT_VOLTAGE) * MAX_PWM_VALUE;
  case VIBRATION_MOTOR:
    return MAX_PWM_VALUE - (VIBRATION_MOTOR_MINIMUM_INPUT_VOLTAGE / VIBRATION_MOTOR_INPUT_VOLTAGE) * MAX_PWM_VALUE;
  default:
    return 0;
  }
}

// Calculate DC motor speed (rotations per second) based on power ratio and
// direction
float get_DC_motor_speed(const float power_ratio) {
  if (power_ratio < 0.0 || power_ratio > 1.0) {
    if (Serial) {
      Serial.printf("Error: Power ratio %.2f out of range (0.0 to 1.0)\n", power_ratio);
    }
    handle_error();
  }

  if (power_ratio == 0.0) {
    return 0.0; // No speed for 0% power
  }

  double x = (double)power_ratio; // Higher precision for calculations
  double speed_rotations_per_second = 11.734452957001 * x * x * x * x * x - 31.542916228504 * x * x * x * x +
                                      30.896714696087 * x * x * x - 13.065680690793 * x * x +
                                      3.494432712543 * x + 0.023545588361;

  speed_rotations_per_second *= DC_MOTOR_INPUT_VOLTAGE / 18.0; // speeds are based on 18V supply, so estimate speeds accordingly
  return (float)speed_rotations_per_second; // Cast result to float
}

// Calculate distance traveled by a motor based on speed and time
inline float distance_traveled_by_motor(const float DC_motor_speed_rotations_per_second,
                           const ulong time_spent_microseconds) {
  return DC_motor_speed_rotations_per_second * (time_spent_microseconds / 1e6); // Convert microseconds to seconds
}

// Calculate motor speed factor based on error and previous speed
float get_motor_speed_factor(const float error, const float previous_speed) {
  float error_magnitude = fabs(error);

  if (previous_speed == 0.0) {
    // Use maximum power to ensure motor starts turning
    return 1.0;
  } else {
    // Adjust speed factor based on error magnitude
    float temp = (error_magnitude < 0.05) ? 0.1 : 1.0;
    return temp;
  }
}

// Put DC motor into sleep mode (High-Z state)
inline void sleep_DC_motor() {
  analogWrite(DC_MOTOR_DIRECTION_PINS[CW], 0);
  analogWrite(DC_MOTOR_DIRECTION_PINS[CCW], 0);
}

// Brake the DC motor (actively stop rotation)
inline void brake_DC_motor() {
  analogWrite(DC_MOTOR_DIRECTION_PINS[CW], 1);
  analogWrite(DC_MOTOR_DIRECTION_PINS[CCW], 1);
}

// Swap motor direction
inline void swap_direction(MOTOR_DIRECTION &direction) {
  direction = (direction == CW) ? CCW : CW;
}

// Map sensor reading to a ratio between 0 and 1
inline float map_ratio(const ushort reading, const ushort threshold, const ushort maxValue) {
  return (float)(map(reading, threshold, maxValue, 0, 10000) / 10000.0);
}

// Map float to a ratio between 0 and 1
inline float map_ratiof(const float value_to_map, const float threshold, const float maxValue) {
  return (float)(map(value_to_map, threshold, maxValue, 0, 10000) / 10000.0);
}

// Update motor position based on speed, time, and direction
inline void
update_motor_position(float &current_position, MOTOR_DIRECTION direction,
                    const float DC_motor_speed_rotations_per_second,
                    const ulong time_spent_microseconds,
                    const MOTOR_DIRECTION motor_tightening_direction) {
  float distance = distance_traveled_by_motor(
      DC_motor_speed_rotations_per_second, time_spent_microseconds);
  current_position += (direction == motor_tightening_direction) ? distance : -distance;
}
