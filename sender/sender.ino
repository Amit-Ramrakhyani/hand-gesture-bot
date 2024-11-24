#include <WiFi.h>
#include <esp_now.h>

// ac:15:18:d4:46:44
#define RECIPIENT_MAC {0xAC, 0x15, 0x18, 0xD4, 0x46, 0x44} // Replace with the receiver's MAC address

typedef struct struct_message {
  char command[10];
} struct_message;


struct_message myData;
uint8_t recipient_mac[6] = RECIPIENT_MAC;

int ledPin = 2;

void setup() {
  Serial.begin(115200);  // Initialize serial communication
  WiFi.mode(WIFI_STA);   // Set ESP32 to station mode

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  pinMode(ledPin, OUTPUT);

  // Add peer (receiver ESP8266)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, recipient_mac, 6);
  peerInfo.channel = 0;  // Use the same channel as the receiver
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void sendGesture(const char* gesture) {
  strcpy(myData.command, gesture);  // Copy the gesture into the struct
  esp_err_t result = esp_now_send(recipient_mac, (uint8_t*)&myData, sizeof(myData));  // Send data via ESP-NOW

  if (result == ESP_OK) {
    Serial.println("Sent successfully");
  } else {
    Serial.println("Send failed");
  }
}

void loop() {
  digitalWrite(ledPin, HIGH);
  if (Serial.available()) {
    String gesture = Serial.readStringUntil('\n');  // Read incoming serial data
    gesture.trim();  // Remove any extra whitespace/newline characters

    if (gesture.length() > 0) {
      sendGesture(gesture.c_str());  // Send the received gesture command
    }
  }
}