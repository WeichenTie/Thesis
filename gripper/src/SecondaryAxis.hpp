#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

class SecondaryAxis : public AccelStepper
{
public:
    SecondaryAxis() : AccelStepper(AccelStepper::FULL4WIRE, IN_1, IN_3, IN_2, IN_4)
    {
        setMaxSpeed(500);
        setAcceleration(10000);
    }

private:
    static constexpr int IN_1 = 21;
    static constexpr int IN_2 = 22;
    static constexpr int IN_3 = 23;
    static constexpr int IN_4 = 25;
};