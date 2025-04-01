/*
  Written by Khoi Hoang
  Built by Khoi Hoang https://github.com/khoih-prog/ESP32_PWM
  Licensed under MIT license

  Adapted by Cooper Mann
  REQUIRED LIBRARY: ESP32_PWM
*/

// SDA = 21, SCL = 22

#include <Wire.h>
#include <Arduino.h>
#include <algorithm>
using namespace std;

#define SLAVE_ADDR 0x55 // Sets address to be looked for

enum State {
  LINEAR,
  TURNING,
  STOP
};

// The current state of the robot
State currentState = STOP;

// Pin values
const int LEFT_MOTOR_PIN = 17;
const int RIGHT_MOTOR_PIN = 16;

// Duty cycles
float currentLeftDutyCycle = 7.5;
float currentRightDutyCycle = 7.5;
float targetLeftDutyCycle = 7.5;
float targetRightDutyCycle = 7.5;


// PID constants
float kp_heading = 0.8;
float kd_heading = 0.005;
float ki_heading = 0.001;

// Other stuff
float lastError = 0.0;
float totalError = 0.0;
float currentHeading = 0.0;
float initialHeading = 0.0;

// These define's must be placed at the beginning before #include "ESP32_PWM.h"
// _PWM_LOGLEVEL_ from 0 to 4
// Don't define _PWM_LOGLEVEL_ > 0. Only for special ISR debugging only. Can hang the system.
#define _PWM_LOGLEVEL_                3

#define USING_MICROS_RESOLUTION true

#include "ESP32_PWM.h"

#define HW_TIMER_INTERVAL_US      20L

uint32_t startMicros = 0;

// Init ESP32 timer 1
ESP32Timer ITimer(1);

// Init ESP32_ISR_PWM
ESP32_PWM ISR_PWM;

bool IRAM_ATTR TimerHandler(void * timerNo)
{
  ISR_PWM.run();

  return true;
}

void receiveEvent(int numBytes) {
  Wire.read();  // Discard the first byte (extra command byte)

  // Get type of message
  int type;
  byte bufferType[4];
    for (int i = 0; i < 4; i++) {
      bufferType[i] = Wire.read();
    }
  memcpy(&type, bufferType, sizeof(type));

  // If it's setting duty cycles, run this to unpack them
  if(type == 1) {
    byte bufferLeft[4], bufferRight[4];
    for (int i = 0; i < 4; i++) {
      bufferLeft[i] = Wire.read();
    }

    for (int i = 0; i < 4; i++) {
      bufferRight[i] = Wire.read();
    }

    memcpy(&targetLeftDutyCycle, bufferLeft, sizeof(targetLeftDutyCycle));  // Convert bytes to float
    memcpy(&targetRightDutyCycle, bufferRight, sizeof(targetRightDutyCycle));  // Convert bytes to float

    // Determine what state to transition into
    if(targetLeftDutyCycle == targetRightDutyCycle) {
      // Stop the robot if 7.5% duty cycle
      if(targetLeftDutyCycle == 7.5) {
        currentState = STOP;
      }
      // Otherwise do heading proportional control
      else {
        initialHeading = currentHeading;
        currentState = LINEAR;
      }
    // Otherwise turn
    } else {
      currentState = TURNING;
    }

  }

  // Send a heading value -180 to 180, unpack and store it in currentHeading
  if(type == 0) {
    byte headingBuffer[4];
    for (int i = 0; i < 4; i++) {
      headingBuffer[i] = Wire.read();
    }
    memcpy(&currentHeading, headingBuffer, sizeof(currentHeading));
  }
}

void update_speeds() {
  // Impose hard limits (5% - 10%)
  currentLeftDutyCycle = std::min(std::max((double) currentLeftDutyCycle, 5.0), 10.0);
  currentRightDutyCycle = std::min(std::max((double) currentRightDutyCycle, 5.0), 10.0);

  // NOTE: DEBUG
  Serial.println("\nVALUES");
  Serial.println(currentLeftDutyCycle);
  Serial.println(currentRightDutyCycle);
  Serial.println("\n");
  
  // Update the PWM signals
  ISR_PWM.modifyPWMChannel_Period(0, LEFT_MOTOR_PIN, 20000, currentLeftDutyCycle);
  ISR_PWM.modifyPWMChannel_Period(1, RIGHT_MOTOR_PIN, 20000, currentRightDutyCycle);
}

void requestEvent() {
  // Convert floats to byte arrays
  byte* ptr1 = (byte*)&currentLeftDutyCycle;
  byte* ptr2 = (byte*)&currentRightDutyCycle;

  Wire.write(ptr1, 4);  // Send first float (4 bytes)
  Wire.write(ptr2, 4);  // Send second float (4 bytes)
}

void update_heading() {
  // Get error
  float error = initialHeading - currentHeading;

  // Derivative componenent
  float deltaError = error - lastError;

  // Integral component
  totalError += error;

  // Calculate the PID for heading correction (constantly updating the target)
  currentLeftDutyCycle = targetLeftDutyCycle + kp_heading * error + kd_heading * deltaError + ki_heading * totalError;
  currentRightDutyCycle = targetRightDutyCycle - kp_heading * error - kd_heading * deltaError - ki_heading * totalError;

  // Also derivative component
  lastError = error;

  // Make sure the motors don't actually stop when doing heading correction
  if(currentLeftDutyCycle > 7.6) {
    std::min(10.0, std::max(7.6, (double) currentLeftDutyCycle));
  }
  if(currentLeftDutyCycle < 7.4) {
    std::max(5.0, std::min(7.4, (double) currentLeftDutyCycle));
  }
  if(currentRightDutyCycle < 7.4) {
    std::max(5.0, std::min(7.4, (double) currentRightDutyCycle));
  }
  if(currentRightDutyCycle > 7.6) {
    std::min(10.0, std::max(7.6, (double) currentRightDutyCycle));
  }
}

// Just set the current to the target (will be sent in update_speeds())
void turn() {
  currentLeftDutyCycle = targetLeftDutyCycle;
  currentRightDutyCycle = targetRightDutyCycle;
}

// Set the PWM signals to neutral (7.5% duty cycle)
void stop() {
  currentLeftDutyCycle = 7.5;
  currentRightDutyCycle = 7.5;
  ISR_PWM.modifyPWMChannel_Period(0, LEFT_MOTOR_PIN, 20000, currentLeftDutyCycle);
  ISR_PWM.modifyPWMChannel_Period(1, RIGHT_MOTOR_PIN, 20000, currentRightDutyCycle);
}

// Setup the microcontroller
void setup()
{
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDR);  // Set ESP32 as slave
  // Define function for when information is requested/transmitted
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  while (!Serial);

  delay(2000);

  // Interval in microsecs, start interrupts
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler))
  {
    startMicros = micros();
    Serial.print(F("Starting ITimer OK, micros() = ")); Serial.println(startMicros);
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));
  
  // Initialize 2 PWM signals at 20 Hz, neutral duty cycle (7.5%)
  ISR_PWM.setPWM_Period(LEFT_MOTOR_PIN, 20000, currentLeftDutyCycle);
  ISR_PWM.setPWM_Period(RIGHT_MOTOR_PIN, 20000, currentRightDutyCycle);

  Serial.println("Armed");
}

void loop() {
  switch(currentState) {
    // Use heading based proportional control
    case LINEAR:
      update_heading();
      update_speeds();
      break;

    // Do basic turning
    case TURNING:
      turn();
      update_speeds();
      break;

    // Stop the motors
    case STOP:
      stop();
      break;

  }
  delay(100);
}