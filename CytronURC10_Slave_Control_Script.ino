#include <CytronMotorDriver.h>
#include <Servo.h>

// Configure Cytron motor drivers
CytronMD motorPort(PWM_DIR, 5, 4);  // Motor 1 (Port)
CytronMD motorStarboard(PWM_DIR, 6, 7);  // Motor 2 (Starboard)

// Define button input pins
const int speedPin = A0;
const int forwardButton = 11;
const int reverseButton = 10;
const int leftButton = 3;
const int rightButton = 2;
const int stopButton = A1;
const int gimbalUpButton = A2;
const int gimbalDownButton = A3;
const int gimbalLeftButton = A4;
const int gimbalRightButton = A5;

// Define servo output pins
const int gimbalLeftRightPin = 13;
const int gimbalUpDownPin = 12;

// Servo objects
Servo gimbalLR;
Servo gimbalUD;

// Gimbal position variables
int gimbalLRPos = 90;  // Start centered
int gimbalUDPos = 90;

void setup() {
  // Set button pins as inputs
  pinMode(forwardButton, INPUT);
  pinMode(reverseButton, INPUT);
  pinMode(leftButton, INPUT);
  pinMode(rightButton, INPUT);
  pinMode(stopButton, INPUT);
  pinMode(gimbalUpButton, INPUT);
  pinMode(gimbalDownButton, INPUT);
  pinMode(gimbalLeftButton, INPUT);
  pinMode(gimbalRightButton, INPUT);

  // Attach servos
  gimbalLR.attach(gimbalLeftRightPin);
  gimbalUD.attach(gimbalUpDownPin);
  
  // Set initial servo positions
  gimbalLR.write(gimbalLRPos);
  gimbalUD.write(gimbalUDPos);
}

void loop() {
  // Read speed control input (0-100%) and convert to motor speed (0-255)
  int speedVal = analogRead(speedPin); 
  int maxSpeed = map(speedVal, 0, 1023, 0, 255);

  // Read button states
  bool stop = digitalRead(stopButton);
  bool forward = digitalRead(forwardButton);
  bool reverse = digitalRead(reverseButton);
  bool left = digitalRead(leftButton);
  bool right = digitalRead(rightButton);
  bool gimbalUp = digitalRead(gimbalUpButton);
  bool gimbalDown = digitalRead(gimbalDownButton);
  bool gimbalLeft = digitalRead(gimbalLeftButton);
  bool gimbalRight = digitalRead(gimbalRightButton);

  // Stop override
  if (stop) {
    motorPort.setSpeed(0);
    motorStarboard.setSpeed(0);
    return;
  }

  // Movement logic based on button states
  if (forward && left) {
    motorPort.setSpeed(maxSpeed / 2);
    motorStarboard.setSpeed(maxSpeed);
  } else if (forward && right) {
    motorPort.setSpeed(maxSpeed);
    motorStarboard.setSpeed(maxSpeed / 2);
  } else if (reverse && left) {
    motorPort.setSpeed(-maxSpeed / 2);
    motorStarboard.setSpeed(-maxSpeed);
  } else if (reverse && right) {
    motorPort.setSpeed(-maxSpeed);
    motorStarboard.setSpeed(-maxSpeed / 2);
  } else if (forward) {
    motorPort.setSpeed(maxSpeed);
    motorStarboard.setSpeed(maxSpeed);
  } else if (reverse) {
    motorPort.setSpeed(-maxSpeed);
    motorStarboard.setSpeed(-maxSpeed);
  } else if (left) {
    motorPort.setSpeed(-maxSpeed);
    motorStarboard.setSpeed(maxSpeed);
  } else if (right) {
    motorPort.setSpeed(maxSpeed);
    motorStarboard.setSpeed(-maxSpeed);
  } else {
    motorPort.setSpeed(0);
    motorStarboard.setSpeed(0);
  }

  // Gimbal control - move servos slowly while button is pressed
  if (gimbalUp && gimbalUDPos < 180) gimbalUDPos++;
  if (gimbalDown && gimbalUDPos > 0) gimbalUDPos--;
  if (gimbalLeft && gimbalLRPos > 0) gimbalLRPos--;
  if (gimbalRight && gimbalLRPos < 180) gimbalLRPos++;

  gimbalLR.write(gimbalLRPos);
  gimbalUD.write(gimbalUDPos);

  delay(10);  // Small delay for button polling stability
}
