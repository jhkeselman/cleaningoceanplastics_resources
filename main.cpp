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

float currentLeftDutyCycle = 7.5;
float currentRightDutyCycle = 7.5;
float targetLeftDutyCycle = 7.5;
float targetRightDutyCycle = 7.5;
float kp = 0.1;
float kp_heading = 0.05;

float lastError = 0.0;
float kd_heading = 0.005;

float totalError = 0.0;
float ki_heading = 0.001;

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
  // int type = Wire.read();
  int type;
  byte bufferType[4];
    for (int i = 0; i < 4; i++) {
      bufferType[i] = Wire.read();
    }
  memcpy(&type, bufferType, sizeof(type));

  // If it's setting duty cycles, run this
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

  // Send a heading value -180 to 180, store it somewhere 
  if(type == 0) {
    byte headingBuffer[4];
    for (int i = 0; i < 4; i++) {
      headingBuffer[i] = Wire.read();
    }
    memcpy(&currentHeading, headingBuffer, sizeof(currentHeading));
  }
}

void update_speeds() {
  double threshold = 0.1;
  // if (std::abs(targetLeftDutyCycle - currentLeftDutyCycle) > threshold) {
  //   currentLeftDutyCycle = currentLeftDutyCycle + kp * (targetLeftDutyCycle - currentLeftDutyCycle);
  // } else {
  //   currentLeftDutyCycle = targetLeftDutyCycle;
  // }

  // if (std::abs(targetRightDutyCycle - currentRightDutyCycle) > threshold) {
  //   currentRightDutyCycle = currentRightDutyCycle + kp * (targetRightDutyCycle - currentRightDutyCycle);
  // } else {
  //   currentRightDutyCycle = targetRightDutyCycle;
  // }  


  if(currentLeftDutyCycle < 5.0) {
    currentLeftDutyCycle = 5.0;
  }
  if(currentLeftDutyCycle > 10.0) {
    currentLeftDutyCycle = 10.0;
  }

  if(currentRightDutyCycle < 5.0) {
    currentRightDutyCycle = 5.0;
  }
  if(currentRightDutyCycle > 10.0) {
    currentRightDutyCycle = 10.0;
  }

  Serial.println("\nVALUES");
  Serial.println(currentLeftDutyCycle);
  Serial.println(currentRightDutyCycle);
  Serial.println("\n");
  
  ISR_PWM.modifyPWMChannel_Period(0, 16, 20000, currentLeftDutyCycle);
  ISR_PWM.modifyPWMChannel_Period(1, 17, 20000, currentRightDutyCycle);
}

void requestEvent() {
  Wire.write("ESP32 OK!");  // Send data to Raspberry Pi
}

void update_heading() {
  float error = initialHeading - currentHeading;

  float deltaError = error - lastError;

  totalError += error;

  currentLeftDutyCycle = targetLeftDutyCycle + kp_heading * error + kd_heading * deltaError + ki_heading * totalError;
  currentRightDutyCycle = targetRightDutyCycle - kp_heading * error - kd_heading * deltaError - ki_heading * totalError;

  lastError = error;

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

void turn() {
  currentLeftDutyCycle = targetLeftDutyCycle;
  currentRightDutyCycle = targetRightDutyCycle;
}

void stop() {
  ISR_PWM.modifyPWMChannel_Period(0, 16, 20000, 7.5);
  ISR_PWM.modifyPWMChannel_Period(1, 17, 20000, 7.5);
}

void setup()
{
  Serial.begin(9600);
  Wire.begin(SLAVE_ADDR);  // Set ESP32 as slave
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  while (!Serial);

  delay(2000);

  // Interval in microsecs
  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler))
  {
    startMicros = micros();
    Serial.print(F("Starting ITimer OK, micros() = ")); Serial.println(startMicros);
  }
  else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));
  
  ISR_PWM.setPWM_Period(16, 20000, 7.5);
  ISR_PWM.setPWM_Period(17, 20000, 7.5);

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

