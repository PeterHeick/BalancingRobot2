#include <Arduino.h>
#include <PID_v1.h>
#include "ESP32.h"
#include "BNO085.h"
#include "robot.h"
#include "Motor.h"

// #define TEST 1

Motor motor(IN1, IN2, ENA, HALL_PIN1, HALL_PIN2);
BNO085 bno085;
euler_t ypr;
float Yaw = 0;

double Setpoint, Input, Output;
double Kp = 2, Ki = 5, Kd = 1;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void IRAM_ATTR hallAISR()
{
  motor.incrementPulseCount();
}

void IRAM_ATTR hallBISR()
{
  // Kan bruges til retningsbestemmelse hvis nødvendigt
}

void setup()
{
  Serial.begin(115200);

  // Initialiser motor
  motor.begin();

  // Try to initialize!
  if (!bno085.begin())
  {
    Serial.printf("Failed to find BNO08x chip\n");
  }
#ifdef TEST
  Serial.printf("BNO08x Found!\n");
#endif

  delay(1000);
  if (bno085.readSensor(ypr))
  {
    Yaw = ypr.yaw;
  }

  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(HALL_PIN1), hallAISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN2), hallBISR, RISING);

  myPID.SetMode(AUTOMATIC);

  // Print CSV header
  Serial.printf("PWM,AdjustedPWM,TargetRPM,ActualRPM, PID, Pitch, Direction\n");
}

void loop()
{
  static int direction = 1;
  static int pwm = 0;
  static unsigned long last = 0;

  unsigned long now = millis();

  if (bno085.readSensor(ypr))
  {
    ypr.pitch += INITBALANCE;

    Setpoint = 0.0;
    Input = ypr.pitch;
    myPID.Compute();
    direction = Output >= 0 ? 1 : -1;
    pwm = (int)std::abs(Output);
#ifdef TEST
    Serial.printf(">> Pid: %.2f, pwm: %d\n", Output, pwm);
#endif
  }

  if (now - last > 100)
  {
    // Få aktuel RPM
    int actualRpm = motor.getActualRpm();
    int adjustedPWM = motor.getAdjustedPwm();
    int targetRpm = motor.getTargetRpm();

    // Print debug-data
    Serial.printf("%d,%d,%d,%d,%.2f, %.2f, %d\n",
                  pwm,
                  adjustedPWM,
                  targetRpm,
                  actualRpm,
                  Output,
                  ypr.pitch,
                  direction);

    motor.setPWM(pwm);
    last = now;
  }
}