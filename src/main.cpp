#include <Arduino.h>
#include <PID_v1.h>
#include "ESP32.h"
#include "BNO085.h"
#include "robot.h"
#include "Motor.h"

Motor motor(IN1, IN2, ENA, HALL_PIN1, HALL_PIN2);
BNO085 bno085;
euler_t ypr;

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
  delay(1000);

  // Setup interrupts
  attachInterrupt(digitalPinToInterrupt(HALL_PIN1), hallAISR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN2), hallBISR, RISING);

  // Print CSV header
  Serial.printf("PWM,AdjustedPWM,TargetRPM,ActualRPM\n");
}

void loop()
{
  static int direction = 1;
  static int pwm = 0;
  static unsigned long last = 0;
  static float prevDt = 0.0;
  static float dt;
  static float filteredDt;
  double Setpoint, Input, Output;

  unsigned long now = millis();

  if (bno085.readSensor(ypr))
  {
    ypr.pitch += INITBALANCE;

    now = micros();
    dt = (now - last) / 1000000.0;
    last = now;
    filteredDt = LOWPASSFILTER(dt, prevDt, ALPHA);
    prevDt = filteredDt;
  }

  if (now - last > 300)
  {
    // Få aktuel RPM
    int actualRpm = motor.getActualRpm();
    int adjustedPWM = motor.getAdjustedPwm();
    int targetRpm = motor.getTargetRpm();

    // Print debug-data
    if (actualRpm > 0)
    {
      Serial.printf("%d,%d,%d,%ld\n",
                    pwm,
                    adjustedPWM,
                    targetRpm,
                    actualRpm);
    }

    // Opdater PWM (sweep op og ned)
    pwm += direction;
    if (pwm >= 255 || pwm <= 0)
    {
      direction = -direction;
    }

    motor.setPWM(pwm);
    last = now;
  }
}