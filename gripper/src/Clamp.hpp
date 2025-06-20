#pragma once

#include <Arduino.h>

class Clamp
{
public:
    Clamp()
    {
    }

    void setup()
    {
        pinMode(PWM_PIN, OUTPUT);
        pinMode(DIR_PIN_A, OUTPUT);
        pinMode(DIR_PIN_B, OUTPUT);
    }

    void clamp()
    {
        digitalWrite(DIR_PIN_A, LOW);
        digitalWrite(DIR_PIN_B, HIGH);
        analogWrite(PWM_PIN, 255);
        delay(3000);
        analogWrite(PWM_PIN, 0);
    }

    void unclamp()
    {
        digitalWrite(DIR_PIN_A, HIGH);
        digitalWrite(DIR_PIN_B, LOW);
        analogWrite(PWM_PIN, 255);
        delay(3000);
        analogWrite(PWM_PIN, 0);
    }

private:
    static const int PWM_PIN = 27;
    static const int DIR_PIN_A = 32;
    static const int DIR_PIN_B = 33;
};