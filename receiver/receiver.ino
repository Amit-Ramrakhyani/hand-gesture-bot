// ----------------------------------------------------------------------------------------------------------------------------------------
// Motor testing code
// ----------------------------------------------------------------------------------------------------------------------------------------

// #include <esp_now.h>
// #include "WiFi.h"

// // Motor 1 Pins
// int motor1Pin1 = 33;
// int motor1Pin2 = 25;
// int enable1Pin = 32;

// // Motor 2 Pins
// int motor2Pin1 = 26;
// int motor2Pin2 = 27;
// int enable2Pin = 14;

// // Built-in LED Pin
// int ledPin = 2;

// void setup() {
//   // Set motor pins as outputs
//   pinMode(motor1Pin1, OUTPUT);
//   pinMode(motor1Pin2, OUTPUT);
//   pinMode(enable1Pin, OUTPUT);

//   pinMode(motor2Pin1, OUTPUT);
//   pinMode(motor2Pin2, OUTPUT);
//   pinMode(enable2Pin, OUTPUT);

//   pinMode(ledPin, OUTPUT);

//   // Start Serial Communication
//   Serial.begin(115200);
//   Serial.println("IoT Car Initialized");
// }


// // Function to move the car forward
// void forward() {
//   Serial.println("Moving Forward");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(enable1Pin, HIGH);
//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, HIGH);

//   digitalWrite(enable2Pin, HIGH);
//   digitalWrite(motor2Pin1, LOW);
//   digitalWrite(motor2Pin2, HIGH);

//   delay(1000);
// }

// // Function to move the car forward
// void backward() {
//   Serial.println("Moving Backward");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(enable1Pin, HIGH);
//   digitalWrite(motor1Pin1, HIGH);
//   digitalWrite(motor1Pin2, LOW);

//   digitalWrite(enable2Pin, HIGH);
//   digitalWrite(motor2Pin1, HIGH);
//   digitalWrite(motor2Pin2, LOW);
//   delay(1000);
// }

// void left() {
//   Serial.println("Moving Left");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(enable1Pin, HIGH);
//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, HIGH);

//   digitalWrite(enable2Pin, HIGH);
//   digitalWrite(motor2Pin1, HIGH);
//   digitalWrite(motor2Pin2, LOW);
//   delay(500);
// }

// void right() {
//   Serial.println("Moving Right");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(enable1Pin, HIGH);
//   digitalWrite(motor1Pin1, HIGH);
//   digitalWrite(motor1Pin2, LOW);

//   digitalWrite(enable2Pin, HIGH);
//   digitalWrite(motor2Pin1, LOW);
//   digitalWrite(motor2Pin2, HIGH);
//   delay(500);
// }


// void loop() {
//   forward();
//   delay(100); // Maintain the forward motion
//   right();
//   delay(100);
//   left();
//   delay(100);
//   backward();
//   delay(100);
// }

// // ----------------------------------------------------------------------------------------------------------------------------------------
// // PWM Motor control code
// // ----------------------------------------------------------------------------------------------------------------------------------------

// // Motor 1 Pins
// int motor1Pin1 = 33;
// int motor1Pin2 = 25;
// int enable1Pin = 32;

// // Motor 2 Pins
// int motor2Pin1 = 26;
// int motor2Pin2 = 27;
// int enable2Pin = 14;

// // Built-in LED Pin
// int ledPin = 2;

// // PWM Configuration
// int pwmFreq = 5000;  // Frequency of PWM signal
// int pwmResolution = 8; // Resolution (8 bits = 0 to 255)

// // Motor Speed (Adjust to control power)
// int msRightForward = 190; 
// int msLeftForward = 160; 

// int msRightBackward = 180; 
// int msLeftBackward = 190;

// void setup() {
//   // Set motor pins as outputs
//   pinMode(motor1Pin1, OUTPUT);
//   pinMode(motor1Pin2, OUTPUT);
//   pinMode(enable1Pin, OUTPUT);

//   pinMode(motor2Pin1, OUTPUT);
//   pinMode(motor2Pin2, OUTPUT);
//   pinMode(enable2Pin, OUTPUT);

//   pinMode(ledPin, OUTPUT);

//   // Configure PWM channels
//   ledcAttach(enable1Pin, pwmFreq, pwmResolution); // Setup Motor 1 enable pin
//   ledcAttach(enable2Pin, pwmFreq, pwmResolution); // Setup Motor 2 enable pin

//   // Start Serial Communication
//   Serial.begin(115200);
//   Serial.println("IoT Car Initialized");
// }

// // Function to move the car forward
// void forward() {
//   Serial.println("Moving Forward");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, HIGH);
//   digitalWrite(motor2Pin1, LOW);
//   digitalWrite(motor2Pin2, HIGH);

//   // Set motor speed
//   ledcWrite(enable1Pin, msRightForward);
//   ledcWrite(enable2Pin, msLeftForward);
// }

// // Function to move the car backward
// void backward() {
//   Serial.println("Moving Backward");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(motor1Pin1, HIGH);
//   digitalWrite(motor1Pin2, LOW);
//   digitalWrite(motor2Pin1, HIGH);
//   digitalWrite(motor2Pin2, LOW);

//   // Set motor speed
//   ledcWrite(enable1Pin, msRightBackward);
//   ledcWrite(enable2Pin, msLeftBackward);
// }

// // Function to turn left
// void turnLeft() {
//   Serial.println("Turning Left");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, HIGH);
//   digitalWrite(motor2Pin1, HIGH);
//   digitalWrite(motor2Pin2, LOW);

//   // Set motor speed
//   ledcWrite(enable1Pin, msRightForward);
//   ledcWrite(enable2Pin, msLeftForward);
// }

// // Function to turn right
// void turnRight() {
//   Serial.println("Turning Right");
//   digitalWrite(ledPin, HIGH); // Indicate action

//   digitalWrite(motor1Pin1, HIGH);
//   digitalWrite(motor1Pin2, LOW);
//   digitalWrite(motor2Pin1, LOW);
//   digitalWrite(motor2Pin2, HIGH);

//   // Set motor speed
//   ledcWrite(enable1Pin, msRightForward);
//   ledcWrite(enable2Pin, msLeftForward);
// }

// // Function to stop the car
// void stopCar() {
//   Serial.println("Stopping");
//   digitalWrite(ledPin, LOW); // Indicate action

//   digitalWrite(motor1Pin1, LOW);
//   digitalWrite(motor1Pin2, LOW);
//   digitalWrite(motor2Pin1, LOW);
//   digitalWrite(motor2Pin2, LOW);

//   // Turn off motors
//   ledcWrite(enable1Pin, 0);
//   ledcWrite(enable2Pin, 0);
// }

// void loop() {
//   forward();
//   delay(2000);

//   backward();
//   delay(2000);

//   turnLeft();
//   delay(2000);

//   turnRight();
//   delay(2000);

//   stopCar();
//   delay(2000);
// }



// ----------------------------------------------------------------------------------------------------------------------------------------
// PWM Motor control code
// ----------------------------------------------------------------------------------------------------------------------------------------

#include <WiFi.h>
#include <esp_now.h>

// Motor 1 Pins
int motor1Pin1 = 33;
int motor1Pin2 = 25;
int enable1Pin = 32;

// Motor 2 Pins
int motor2Pin1 = 26;
int motor2Pin2 = 27;
int enable2Pin = 14;

// Built-in LED Pin
int ledPin = 2;

// PWM Configuration
int pwmFreq = 5000;  // Frequency of PWM signal
int pwmResolution = 8; // Resolution (8 bits = 0 to 255)

// Motor Speed (Adjust to control power)
int msRightForward = 190; 
int msLeftForward = 160; 

int msRightBackward = 180; 
int msLeftBackward = 180;

int msRightLeft = 150; 
int msLeftLeft = 150; 

int msRightRight = 150; 
int msLeftRight = 150;

typedef struct struct_message {
  char command[10];
} struct_message;

struct_message myData;

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Command received: ");
  Serial.println(myData.command);

  if (strcmp(myData.command, "forward") == 0) {
    forward();
  } else if (strcmp(myData.command, "backward") == 0) {
    backward();
  } else if (strcmp(myData.command, "left") == 0) {
    left();
  } else if (strcmp(myData.command, "right") == 0) {
    right();
  } else {
    stop();
  }
}

void setup() {

  // Start Serial Communication
  Serial.begin(115200);
  Serial.println("IoT Car Initialized");
  WiFi.mode(WIFI_STA);

  // Set motor pins as outputs
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  pinMode(ledPin, OUTPUT);

  // Configure PWM channels
  ledcAttach(enable1Pin, pwmFreq, pwmResolution); // Setup Motor 1 enable pin
  ledcAttach(enable2Pin, pwmFreq, pwmResolution); // Setup Motor 2 enable pin
  
  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback function
  esp_now_register_recv_cb(onDataRecv);

}

// Function to move the car forward
void forward() {
  Serial.println("Moving Forward");
  digitalWrite(ledPin, HIGH); // Indicate action

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  // Set motor speed
  ledcWrite(enable1Pin, msRightForward);
  ledcWrite(enable2Pin, msLeftForward);
}

// Function to move the car backward
void backward() {
  Serial.println("Moving Backward");
  digitalWrite(ledPin, HIGH); // Indicate action

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  // Set motor speed
  ledcWrite(enable1Pin, msRightBackward);
  ledcWrite(enable2Pin, msLeftBackward);
}

// Function to turn left
void left() {
  Serial.println("Turning Left");
  digitalWrite(ledPin, HIGH); // Indicate action

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);

  // Set motor speed
  ledcWrite(enable1Pin, msRightLeft);
  ledcWrite(enable2Pin, msLeftLeft);
}

// Function to turn right
void right() {
  Serial.println("Turning Right");
  digitalWrite(ledPin, HIGH); // Indicate action

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);

  // Set motor speed
  ledcWrite(enable1Pin, msRightRight);
  ledcWrite(enable2Pin, msLeftRight);
}

// Function to stop the car
void stop () {
  Serial.println("Stopping");
  digitalWrite(ledPin, LOW); // Indicate action

  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);

  // Turn off motors
  ledcWrite(enable1Pin, 0);
  ledcWrite(enable2Pin, 0);
}

void loop() {
  // forward();
  // delay(2000);

  // backward();
  // delay(2000);

  // turnLeft();
  // delay(2000);

  // turnRight();
  // delay(2000);

  // stopCar();
  // delay(2000);
}

