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

#define SLAVE_ADDR 0x55 // Sets address to be looked for

float leftDutyCycle = 0.0;
float rightDutyCycle = 0.0;

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

float current_duty_cycle = 7.5;

bool IRAM_ATTR TimerHandler(void * timerNo)
{
  ISR_PWM.run();

  return true;
}

void receiveEvent(int numBytes) {
  Wire.read();  // Discard the first byte (extra command byte)
  
  byte bufferLeft[4], bufferRight[4];
  for (int i = 0; i < 4; i++) {
    bufferLeft[i] = Wire.read();
  }

  for (int i = 0; i < 4; i++) {
    bufferRight[i] = Wire.read();
  }

  memcpy(&leftDutyCycle, bufferLeft, sizeof(leftDutyCycle));  // Convert bytes to float
  memcpy(&rightDutyCycle, bufferRight, sizeof(rightDutyCycle));  // Convert bytes to float

  // Update speeds
  ISR_PWM.modifyPWMChannel_Period(0, 16, 20000, leftDutyCycle);
  ISR_PWM.modifyPWMChannel_Period(1, 17, 20000, rightDutyCycle);
}

void requestEvent() {
  Wire.write("ESP32 OK!");  // Send data to Raspberry Pi
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
  delay(100);
}

