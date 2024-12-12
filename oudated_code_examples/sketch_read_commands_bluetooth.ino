#include "BluetoothSerial.h"
BluetoothSerial SerialBT;

void setup() {
  Serial.begin(115200);                // Start the Serial Monitor
  SerialBT.begin("ESP32_BT_Controller"); // Set the Bluetooth device name
  Serial.println("ESP32 Bluetooth Device is Ready to Pair");
}

void loop() {
  if (SerialBT.available()) {          // Check if data is available
    char incomingChar = SerialBT.read(); // Read the incoming character
    Serial.print("Received: ");        // Print received character to console
    Serial.println(incomingChar);

    // Respond based on the command received
    switch (incomingChar) {
      case 'F': // Move Forward
        Serial.println("Moving Forward");
        // Add code to control motors here
        break;
      case 'B': // Move Backward
        Serial.println("Moving Backward");
        // Add code to control motors here
        break;
      case 'L': // Turn Left
        Serial.println("Turning Left");
        // Add code to control motors here
        break;
      case 'R': // Turn Right
        Serial.println("Turning Right");
        // Add code to control motors here
        break;
      default:
        Serial.println("Unknown Command");
        break;
    }
  }
}
