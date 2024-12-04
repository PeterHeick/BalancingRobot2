// Description: Robot configuration file
// Max pitch for balancing robot
#ifndef robot_h
#define robot_h

#include <Arduino.h>

#define MAXPITCH 5

// PID konstanter
#define KP 50.0
#define KI 220.0
#define KD 0.4

#define INITBALANCE 0

#define DIRECTION_MAX_LENGTH 10

#define ALPHA 0.1
#define LOWPASSFILTER(currentValue, previousValue, alpha) ((alpha) * (currentValue) + (1 - (alpha)) * (previousValue))

struct RobotConfig
{
  int velocity;
  char direction[DIRECTION_MAX_LENGTH];
  float kp, ki, kd;
  int minvelocity;
  int maxpitch;
};
#endif