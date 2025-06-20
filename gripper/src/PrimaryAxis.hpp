#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

class PrimaryAxis : public AccelStepper
{
public:
    PrimaryAxis() : AccelStepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN)
    {
        setMaxSpeed(5000);
        setAcceleration(10000);
    }

    void setup()
    {
        pinMode(M0, OUTPUT);
        pinMode(M1, OUTPUT);
        pinMode(M2, OUTPUT);
        digitalWrite(M0, HIGH);
        digitalWrite(M1, HIGH);
        digitalWrite(M2, HIGH);
    }

private:
    static constexpr int M0 = 19;
    static constexpr int M1 = 18;
    static constexpr int M2 = 17;
    static constexpr int DIR_PIN = 15;
    static constexpr int STEP_PIN = 16;
};