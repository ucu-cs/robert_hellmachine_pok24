//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

// RemoteXY select connection mode and include library 
#define REMOTEXY_MODE__WIFI_CLOUD

#include <WiFi.h>

// RemoteXY connection settings 
#define REMOTEXY_WIFI_SSID "UCU_Guest"
#define REMOTEXY_WIFI_PASSWORD ""
#define REMOTEXY_CLOUD_SERVER "cloud.remotexy.com"
#define REMOTEXY_CLOUD_PORT 6376
#define REMOTEXY_CLOUD_TOKEN "e0586d5cdbfa4b147742ae2faade9cf7"

#include <RemoteXY.h>

// RemoteXY GUI configuration  
#pragma pack(push, 1)  
uint8_t RemoteXY_CONF[] =   // 62 bytes
  { 255,4,0,0,0,55,0,19,0,0,0,82,111,98,101,114,116,0,31,1,
  106,200,1,1,4,0,1,35,55,33,33,0,1,31,0,1,35,109,33,33,
  0,1,31,0,1,61,82,33,33,0,1,31,0,1,7,81,33,33,0,1,
  31,0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t button_01; // =1 if button pressed, else =0
  uint8_t button_02; // =1 if button pressed, else =0
  uint8_t button_03; // =1 if button pressed, else =0
  uint8_t button_04; // =1 if button pressed, else =0

  // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0
} RemoteXY;
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

// Define the LED pin
#define LED_PIN 2  // Pin for the LED, change to the pin where the LED is connected

void setup() {
  Serial.begin(115200);                // Start the Serial Monitor
  RemoteXY_Init();  
  pinMode(LED_PIN, OUTPUT);  // Initialize the LED pin as output
}

void loop() { 
  RemoteXY_Handler();  // Handle the RemoteXY data

  // Check button 1: Turn LED ON
  if (RemoteXY.button_01) {
    digitalWrite(LED_PIN, HIGH);  // Turn LED ON
    Serial.println("F");
    RemoteXY_delay(300);
  }

  // Check button 2: Turn LED OFF
  if (RemoteXY.button_02) {
    digitalWrite(LED_PIN, LOW);   // Turn LED OFF
    Serial.println("B");
    RemoteXY_delay(300);
  }

  // Check button 3: Blink LED quickly
  if (RemoteXY.button_03) {
    Serial.println("R");
    digitalWrite(LED_PIN, HIGH);  // Turn LED ON
    RemoteXY_delay(100);          // Wait for 100ms
    digitalWrite(LED_PIN, LOW);   // Turn LED OFF
    RemoteXY_delay(100);          // Wait for 100ms
  }

  // Check button 4: Blink LED slowly
  if (RemoteXY.button_04) {
    Serial.println("L");
    digitalWrite(LED_PIN, HIGH);  // Turn LED ON
    RemoteXY_delay(500);          // Wait for 500ms
    digitalWrite(LED_PIN, LOW);   // Turn LED OFF
    RemoteXY_delay(500);          // Wait for 500ms
  }

  // Use RemoteXY_delay() instead of delay()
  RemoteXY_delay(50);  // Add a small delay to avoid rapid toggling
}
