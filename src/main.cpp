#include <Arduino.h>

// Include PS3 Controller library
#include <Ps3Controller.h>
 
// Define motor driver pins
#define PWMA_PIN 16
#define AIN1_PIN 14
#define AIN2_PIN 12
#define PWMB_PIN 27
#define BIN1_PIN 17
#define BIN2_PIN 13
#define ENA_PIN  2
#define ENB_PIN  4
 
// Define PWM Parameters
const int motorFreq = 15000;
const int motorResolution = 8;
 
// Define channels for each motor
const int motorAChannel = 3;
const int motorBChannel = 4;
 
// Variables for Motor PWM values
int motorAPWM = 0;
int motorBPWM = 0;
 
// Variables for motor direction - true=forward
bool motorDir = true;
 
// Variables for joystick values
int rightX = 0;
int rightY = 0;

// Motor movement function
void moveMotors(int mtrAspeed, int mtrBspeed, bool mtrdir) {
  // Set direction pins
  if (!mtrdir) {
    // Move in reverse
    digitalWrite(AIN1_PIN, HIGH);
    digitalWrite(AIN2_PIN, LOW);
    digitalWrite(BIN1_PIN, HIGH);
    digitalWrite(BIN2_PIN, LOW);
 
  } else {
    // Move Forward
    digitalWrite(AIN1_PIN, LOW);
    digitalWrite(AIN2_PIN, HIGH);
    digitalWrite(BIN1_PIN, LOW);
    digitalWrite(BIN2_PIN, HIGH);
  }
 
  // Drive motors with PWM
  ledcWrite(motorAChannel, mtrAspeed);
  ledcWrite(motorBChannel, mtrBspeed);
}

// Callback Function
void notify() {
 
  // Get Joystick value
  rightX = (Ps3.data.analog.stick.rx);
  rightY = (Ps3.data.analog.stick.ry);
 
  //Determine direction from Y axis position
  if (rightY < 0) {
    // Direction is forward
    motorDir = true;
  } else {
    // Direction is reverse
    motorDir = false;
  }
 
  // Convert joystick values to positive 0 - 255
  int speedX = (abs(rightX) * 2);
  int speedY = (abs(rightY) * 2);
 
  // Factor in the X axis value to determine motor speeds (assume Motor A is Left motor going forward)
  if (rightX < -10) {
    // Motor B faster than Motor A
    motorAPWM = speedY - speedX;
    motorBPWM = speedY + speedX;
 
  } else if (rightX > 10) {
    // Motor A faster than Motor B
    motorAPWM = speedY + speedX;
    motorBPWM = speedY - speedX;
 
  } else {
    // Control is in middle, both motors same speed
    motorAPWM = speedY;
    motorBPWM = speedY;
  }
 
  // Ensure that speed values remain in range of 0 - 255
  motorAPWM = constrain(motorAPWM, 0, 255);
  motorBPWM = constrain(motorBPWM, 0, 255);
 
  // Drive the motors
  moveMotors(motorAPWM, motorBPWM, motorDir);
 
  // Print to Serial Monitor
  Serial.print("X value = ");
  Serial.print(rightX);
  Serial.print(" - Y value = ");
  Serial.print(rightY);
  Serial.print(" - Motor A = ");
  Serial.print(motorAPWM);
  Serial.print(" - Motor B = ");
  Serial.println(motorBPWM);
}
 
// On Connection function
void onConnect() {
  // Print to Serial Monitor
  Serial.println("Connected.");
}
 
 
void setup() {
 
  // Setup Serial Monitor for testing
  Serial.begin(115200);
 
  // Define Callback Function
  Ps3.attach(notify);
  // Define On Connection Function
  Ps3.attachOnConnect(onConnect);
  // Emulate console as specific MAC address (change as required)
  Ps3.begin("00:00:00:00:00:00");
 
  // Set motor controller pins as outputs
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(ENA_PIN, OUTPUT);
  pinMode(ENB_PIN, OUTPUT);
 
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(ENB_PIN, HIGH);

  // Set channel Parameters for each motor
  ledcSetup(motorAChannel, motorFreq, motorResolution);
  ledcSetup(motorBChannel, motorFreq, motorResolution);
 
  // Attach Motor PWM pins to corresponding channels
  ledcAttachPin(PWMA_PIN, motorAChannel);
  ledcAttachPin(PWMB_PIN, motorBChannel);
 
  // Print to Serial Monitor
  Serial.println("Ready.");
}
 
void loop() {
  if (!Ps3.isConnected())
    return;
  delay(2000);
}