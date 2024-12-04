#include "Motor.h"

Motor::Motor(int pinIN1, int pinIN2, int pinENA, int hallPin1, int hallPin2,
             float gearRatio, int countsPerMotorRev) : _pinIN1(pinIN1),
                                                       _pinIN2(pinIN2),
                                                       _pinENA(pinENA),
                                                       _hallPin1(hallPin1),
                                                       _hallPin2(hallPin2),
                                                       _gearRatio(gearRatio),
                                                       _countsPerMotorRev(countsPerMotorRev),
                                                       _pwmFrekvens(5000),
                                                       _pwmKanal(0),
                                                       _pwmResolution(8),
                                                       _pwmMax(255),
                                                       _rpmMin(5),
                                                       _rpmMax(234),
                                                       _minMeasurementTime(90),
                                                       _pulseCount(0),
                                                       _adjustedPwm(0),
                                                       _targetRpm(0),
                                                       _actualRpm(0),
                                                       _isAccelerating(true),
                                                       _startMeasurementTime(0)
{
}

void Motor::begin()
{
    // Motor pin setup
    pinMode(_pinIN1, OUTPUT);
    pinMode(_pinIN2, OUTPUT);
    ledcSetup(_pwmKanal, _pwmFrekvens, _pwmResolution);
    ledcAttachPin(_pinENA, _pwmKanal);

    // Hall sensor setup
    pinMode(_hallPin1, INPUT_PULLUP);
    pinMode(_hallPin2, INPUT_PULLUP);

    // Start motoren fremad
    setDirection(true);
    setPWM(0);

    _startMeasurementTime = millis();
}

void Motor::setDirection(bool forward)
{
    digitalWrite(_pinIN1, forward ? HIGH : LOW);
    digitalWrite(_pinIN2, forward ? LOW : HIGH);
}

//   pwm     adjustedPwm    targetRpm
// 0 - 255     30 - 255      5 - 234
//
void Motor::setPWM(int pwm)
{
    int diff;
    static int prevPwm = 0;
    static int diffArray[256] = {0};
    static int prt = 0;

    int actual = getActualRpm();
    diff = _targetRpm - actual;
    Serial.printf(">> TargetRpm: %d, Actual: %d, diff: %d\n", _targetRpm, actual, diff);
    // diffArray[prevPwm] = diff;
    diffArray[prevPwm] = (int)LOWPASSFILTER(diff, diffArray[prevPwm], ALPHA);

    _targetRpm = (int)map(pwm, 0, _pwmMax, _rpmMin, _rpmMax);
    Serial.printf(">> TargetRpm: %d = pwm: %d, _pwmMax: %d, _rpmMin: %d, _rpmMax: %d\n", _targetRpm, pwm, _pwmMax, _rpmMin, _rpmMax);

    _adjustedPwm = adjustPWM(pwm, pwm >= prevPwm);
    ledcWrite(_pwmKanal, _adjustedPwm + diffArray[pwm]);
    resetPulseCount();
    prevPwm = pwm;

    if (prt++ % 100 == 0)
    {
        for (int i = 0; i <= 255; i++)
        {
            Serial.printf("%d, %d\n", i, diffArray[i]);
        }
        Serial.printf("\n");
    }
}

int Motor::getActualRpm()
{
    updateRPM();
    return _actualRpm;
}

int Motor::getAdjustedPwm() const
{
    return _adjustedPwm;
}

int Motor::getTargetRpm() const
{
    return _targetRpm;
}

void Motor::incrementPulseCount()
{
    _pulseCount++;
}

void Motor::resetPulseCount()
{
    _pulseCount = 0;
    _startMeasurementTime = millis();
}

int Motor::adjustPWM(int pwmA, bool isAccelerating)
{
    int pwmB = 0;

    if (pwmA == 0)
    {
        return 0;
    }
    if (isAccelerating)
    {
        pwmB = map(pwmA, 1, 255, 30, 255);
    }
    else
    {
        pwmB = map(pwmA, 1, 255, 25, 255);
    }

    return pwmB;
}

void Motor::updateRPM()
{
    unsigned long timepassed = millis() - _startMeasurementTime;

    if (timepassed > _minMeasurementTime)
    {
        float pulsesPerSecond = (float)_pulseCount * (1000.0 / timepassed);
        float motorRPM = (pulsesPerSecond * 60.0) / _countsPerMotorRev;
        _actualRpm = (int)(motorRPM / _gearRatio);
    }
}