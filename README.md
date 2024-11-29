# Haptic-Feedback-System-for-Transradial-Amputees

# USAGE

REQUIRES ARDUINO IDE AND 2 ESP32-WROOM-32E development boards from FreeNove

install the 41x library ("41x_ESP_library") by zipping the folder and using the "add zip library" feature of the arduino IDE under "sketch"

# FUNCTIONALITY
the BLE_server code is uploaded to the ESP32 for the hand, which takes readings and sends data
the BLE_client code is uploaded to the ESP32 for the sleeve, which drives the DC motor and vibration motors
