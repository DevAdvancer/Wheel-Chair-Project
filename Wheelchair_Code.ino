// Bluetooth Control 
#include "BluetoothSerial.h"
#include <Arduino.h>

BluetoothSerial serialBT;

char btSignal;
int Speed = 100;

int R1PWM = 19;
int R2PWM = 21;
int L1PWM = 23;
int L2PWM = 22;

#define R1 0
#define R2 1
#define L1 2
#define L2 3

void setup() {
  Serial.begin(115200);
  serialBT.begin("Wheel Chair Project");
  pinMode(16, HIGH);
  pinMode(32, HIGH);
  pinMode(R1PWM, OUTPUT);
  pinMode(R2PWM, OUTPUT);
  pinMode(L1PWM, OUTPUT);
  pinMode(L2PWM, OUTPUT);

  ledcSetup(R1, 5000, 8);
  ledcAttachPin(R1PWM, R1);
  ledcSetup(R2, 5000, 8);
  ledcAttachPin(R2PWM, R2);
  ledcSetup(L1, 5000, 8);
  ledcAttachPin(L1PWM, L1);
  ledcSetup(L2, 5000, 8);
  ledcAttachPin(L2PWM, L2);
}

void driveMotors(int speedR1, int speedR2, int speedL1, int speedL2) {
  ledcWrite(R1, speedR1);
  ledcWrite(R2, speedR2);
  ledcWrite(L1, speedL1);
  ledcWrite(L2, speedL2);
}

void applyBrakes() {
  driveMotors(0, 0, 0, 0);  // apply brakes
}

void loop() {
  if (!serialBT.connected()) {
    applyBrakes();
    Serial.println("Bluetooth connection lost. Applying brakes.");
    delay(1000); // Optional delay to avoid rapid braking
    return;
  }

  while (serialBT.available()) {
    btSignal = serialBT.read();

    switch (btSignal) {
       case '0':
        Speed = 100;
        break;
      case '1':
        Speed = 110;
        break;
      case '2':
        Speed = 120;
        break;
      case '3':
        Speed = 130;
        break;
      case '4':
        Speed = 140;
        break;
      case '5':
        Speed = 150;
        break;
      case '6':
        Speed = 180;
        break;
      case '7':
        Speed = 200;
        break;
      case '8':
        Speed = 220;
        break;
      case '9':
        Speed = 240;
        break;
      case 'q':
        Speed = 255;
        break;
      case 'B':
        driveMotors(Speed, 0, Speed, 0);  // forward
        break;
      case 'F':
        driveMotors(0, Speed, 0, Speed);  // backward
        break;
      case 'L':
        driveMotors(0, Speed, Speed, 0);  // left
        break;
      case 'R':
        driveMotors(Speed, 0, 0, Speed);  // right
        break;
      case 'S':
        driveMotors(0, 0, 0, 0);  // stop
        break;
      case 'I':
        driveMotors(Speed, 0, 0, 0);  // forward right
        break;
      case 'J':
        driveMotors(0, Speed, 0, 0);  // backward right
        break;
      case 'G':
        driveMotors(0, 0, Speed, 0);  // forward left
        break;
      case 'H':
        driveMotors(0, 0, 0, Speed);  // backward left
        break;
    }
  }
}

/*=====================================================================================================================================================================================================================================*/
// Joystick Control 
// const int joystickXPin = 34; // Connect X-axis of joystick to pin 34
// const int joystickYPin = 35; // Connect Y-axis of joystick to pin 35
// const int joystickButtonPin = 32; // Connect the middle button of the joystick to pin 32

// const int motor1Pin1 = 4; // Motor 1 positive terminal
// const int motor1Pin2 = 5; // Motor 1 negative terminal

// const int motor2Pin1 = 14; // Motor 2 positive terminal
// const int motor2Pin2 = 15; // Motor 2 negative terminal

// const int maxSpeed = 160; // Maximum speed for gradual increase
// const int minSpeed = 100; // Minimum speed

// unsigned long lastMovementTime = 0;
// unsigned long accelerationTime = 120000; // 2 minutes (in milliseconds)

// void setup() {
//   pinMode(joystickXPin, INPUT);
//   pinMode(joystickYPin, INPUT);
//   pinMode(joystickButtonPin, INPUT_PULLUP);

//   pinMode(motor1Pin1, OUTPUT);
//   pinMode(motor1Pin2, OUTPUT);
//   pinMode(motor2Pin1, OUTPUT);
//   pinMode(motor2Pin2, OUTPUT);

//   // Set motor pins to LOW initially
//   ledcWrite(motor1Pin1, LOW);
//   ledcWrite(motor1Pin2, LOW);
//   ledcWrite(motor2Pin1, LOW);
//   ledcWrite(motor2Pin2, LOW);
// }

// void loop() {
//   int xAxisValue = analogRead(joystickXPin);
//   int yAxisValue = analogRead(joystickYPin);
//   int buttonState = digitalRead(joystickButtonPin);

//   // Map joystick values to motor speeds
//   int motor1Speed = map(xAxisValue, 0, 1023, -maxSpeed, maxSpeed);
//   int motor2Speed = map(yAxisValue, 0, 1023, -maxSpeed, maxSpeed);

//   // Check the button state
//   if (buttonState == LOW) { // Button is pressed
//     stopMotors();
//     lastMovementTime = millis(); // Reset the timer when the button is pressed
//   } else {
//     // Control the motors with gradual speed change
//     controlMotorsWithAcceleration(motor1Speed, motor2Speed);
//   }
// }

// void controlMotorsWithAcceleration(int speed1, int speed2) {
//   unsigned long elapsedTime = millis() - lastMovementTime;

//   // Gradually increase the speed from minSpeed to maxSpeed over accelerationTime
//   int currentSpeed1 = map(elapsedTime, 0, accelerationTime, minSpeed, maxSpeed);
//   int currentSpeed2 = map(elapsedTime, 0, accelerationTime, minSpeed, maxSpeed);

//   // Ensure the speed does not exceed maxSpeed
//   currentSpeed1 = min(currentSpeed1, maxSpeed);
//   currentSpeed2 = min(currentSpeed2, maxSpeed);

//   // Motor 1
//   if (speed1 > 0) {
//     ledcWrite(motor1Pin1, HIGH);
//     ledcWrite(motor1Pin2, LOW);
//   } else if (speed1 < 0) {
//     ledcWrite(motor1Pin1, LOW);
//     ledcWrite(motor1Pin2, HIGH);
//   } else {
//     ledcWrite(motor1Pin1, LOW);
//     ledcWrite(motor1Pin2, LOW);
//   }

//   // Motor 2
//   if (speed2 > 0) {
//     ledcWrite(motor2Pin1, HIGH);
//     ledcWrite(motor2Pin2, LOW);
//   } else if (speed2 < 0) {
//     ledcWrite(motor2Pin1, LOW);
//     ledcWrite(motor2Pin2, HIGH);
//   } else {
//     ledcWrite(motor2Pin1, LOW);
//     ledcWrite(motor2Pin2, LOW);
//   }

//   // If the joystick is at rest, reset the timer
//   if (speed1 == 0 && speed2 == 0) {
//     lastMovementTime = millis();
//   }
// }

// void stopMotors() {
//   ledcWrite(motor1Pin1, LOW);
//   ledcWrite(motor1Pin2, LOW);
//   ledcWrite(motor2Pin1, LOW);
//   ledcWrite(motor2Pin2, LOW);
// }
