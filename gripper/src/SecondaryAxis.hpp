#pragma once

#include <Arduino.h>
#include <AccelStepper.h>

class SecondaryAxis : public AccelStepper
{
public:
    SecondaryAxis() : AccelStepper(AccelStepper::FULL4WIRE, IN_1, IN_3, IN_2, IN_4)
    {
        setMaxSpeed(600);
        setAcceleration(10000);
    }

    float normalizeAngle(float angle)
    {
        while (angle > PI)
            angle -= 2.0f * PI;
        while (angle < -PI)
            angle += 2.0f * PI;
        return angle;
    }

    void setTargetPositionRadians(float targetRadians)
    {
        float currentRadians = currentPosition() / STEPS_PER_RADIAN;

        float delta = normalizeAngle(targetRadians - currentRadians);

        long targetSteps = currentPosition() + delta * STEPS_PER_RADIAN;
        moveTo(targetSteps);
    }

private:
    static constexpr int IN_1 = 0;
    static constexpr int IN_2 = 2;
    static constexpr int IN_3 = 4;
    static constexpr int IN_4 = 5;

    static constexpr float STEPS_PER_RADIAN = 2048.0f / (2.0f * PI);
};