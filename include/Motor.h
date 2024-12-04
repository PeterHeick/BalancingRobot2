#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "robot.h"

class Motor
{
private:
    // Pin konfiguration
    const int _pinIN1;
    const int _pinIN2;
    const int _pinENA;
    const int _hallPin1;
    const int _hallPin2;

    // Motor specifikationer
    const float _gearRatio;
    const int _countsPerMotorRev;

    // PWM konfiguration
    const int _pwmFrekvens;
    const int _pwmKanal;
    const int _pwmResolution;
    const int _pwmMax;
    const int _rpmMin;
    const int _rpmMax;
    const int _minMeasurementTime;

    // Motor tilstand
    volatile long _pulseCount;
    int _adjustedPwm;
    int _targetRpm;
    int _actualRpm;
    bool _isAccelerating;

    // Timing variabler (flyttet ned for at matche initialiseringsrækkefølgen)
    unsigned long _startMeasurementTime;
    unsigned long _currentTime;

    // Private metoder
    int adjustPWM(int pwmA, bool isAccelerating);
    void updateRPM();

public:
    Motor(int pinIN1, int pinIN2, int pinENA, int hallPin1, int hallPin2,
          float gearRatio = 43.7, int countsPerMotorRev = 16);

    void begin();
    void setDirection(bool forward);
    void setPWM(int pwm);
    int getActualRpm();
    int getAdjustedPwm() const;
    int getTargetRPM() const;
    void incrementPulseCount();
    void resetPulseCount();
    int getTargetRpm() const;
};

#endif // MOTOR_H