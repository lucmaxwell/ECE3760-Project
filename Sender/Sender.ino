#include "FastLED.h"
#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h>

#define DATA_PIN 26
#define NUM_LEDS 12
#define CHANNEL 1

int xPin = 36; // connect joystick X axis to analog input A0
int yPin = 39; // connect joystick Y axis to analog input A1

// Set receiver (slave) ESP32 MAC address
uint8_t receiverMacAddress[] = { 0x40, 0x22, 0xD8, 0xEA, 0x87, 0xA4 };

// Define data structure for communication
typedef struct struct_message {
  int A;
  int B;
} struct_message;

// Create instance of data structure
struct_message message;


// create an array of CRGB objects to represent the LEDs
// CRGB leds[NUM_LEDS];

void setup() { 
  Serial.begin(115200); // initialize serial communication at 9600 bits per second

  // Initialize WiFi module
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initialize ESP-Now
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-Now");
    return;
  }

  // Register peer ESP32 (slave)
  esp_now_peer_info_t peerInfo;
  memset(&peerInfo, 0, sizeof(peerInfo));
  peerInfo.channel = CHANNEL;
  peerInfo.encrypt = false;
  memcpy(peerInfo.peer_addr, receiverMacAddress, sizeof(receiverMacAddress));
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
}

void loop() {
  int xValue = analogRead(xPin); // read the X axis value
  int yValue = analogRead(yPin); // read the Y axis value
  
  // map the joystick values (0 to 1023)
  // float xMapped = map(xValue, 0, 1023, 0, 1);
  // float yMapped = map(yValue, 0, 1023, 0, 1);

  message.A = xValue;
  message.B = yValue;
  // Serial.print("   x");
  // Serial.print(xMapped);
  // Serial.print("   y");
  // Serial.print(yMapped);


  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&message, sizeof(message));
  if (result == ESP_OK) {
    Serial.println("Message sent to slave");
  } else {
    Serial.println("Error sending message to slave");
  }

  delay(1000); // wait for 1000 milliseconds before reading again



  // // Turn the first led red for 1 second
  // leds[0] = CRGB::Red; 
  // FastLED.show();
  // delay(1000);
  
  // // Set the first led back to black for 1 second
  // leds[0] = CRGB::Black;
  // FastLED.show();
  // delay(1000);
}

