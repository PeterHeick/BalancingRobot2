#ifndef ESP32_H
#define ESP32_H
// Description: Defines for ESP32 pins.

// Gyro: MPU6050
#define SDA 21    // Blå
#define SCL 22    // Gul

// HC-SR04 ultralydssensor:
// Trig på HC-SR04 til GPIO32 på ESP32 Output.
// Echo på HC-SR04 til GPIO33 på ESP32 Input.
#define TRIGGER 32
#define ECHO 33

// Motor controller: L298N
// Motor 1:

// Pin definitioner
const int IN1 = 26;
const int IN2 = 27;
const int ENA = 25;
const int HALL_PIN1 = 34;
const int HALL_PIN2 = 35;

// Motor 2:
#define ENB 18
#define IN3 16
#define IN4 17

#endif