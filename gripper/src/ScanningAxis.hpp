#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <PID_v1.h>

class ScanningAxis
{
public:
    ScanningAxis()
    {
    }

    void setup()
    {
        encoder = new Encoder(EN_A, EN_B);
        pid = new PID(&input, &output, &setpoint, 0.04, 0, 0.005, DIRECT);
        pid->SetMode(AUTOMATIC);
        pid->SetOutputLimits(-175, 175);
        pinMode(PWM_PIN, OUTPUT);
        pinMode(DIR_PIN_A, OUTPUT);
        pinMode(DIR_PIN_B, OUTPUT);
    }

    void calibrate()
    {
        digitalWrite(DIR_PIN_A, LOW);
        digitalWrite(DIR_PIN_B, HIGH);
        analogWrite(PWM_PIN, 45);
        delay(1000);
        encoder->readAndReset();
        stop();
    };

    void engage()
    {
        digitalWrite(DIR_PIN_A, HIGH);
        digitalWrite(DIR_PIN_B, LOW);
        analogWrite(PWM_PIN, 45);
    }

    void disengage()
    {
        setpoint = 0;

        unsigned long start = millis();
        while (millis() - start < 250)
        {
            input = encoder->read();
            pid->Compute();

            if (abs(setpoint - input) < 50)
                break;

            if (output > 0)
            {
                digitalWrite(DIR_PIN_A, HIGH);
                digitalWrite(DIR_PIN_B, LOW);
                analogWrite(PWM_PIN, output);
            }
            else
            {
                digitalWrite(DIR_PIN_A, LOW);
                digitalWrite(DIR_PIN_B, HIGH);
                analogWrite(PWM_PIN, -output);
            }
            delay(2);
        }
        stop();
    }

    void stop()
    {
        analogWrite(PWM_PIN, 0);
    }

    double getDistanceToCenter()
    {
        long encoderPosition = encoder->read();
        double theta = (double)encoderPosition / (double)COUNTS_PER_REVOLUTION * 2.0 * PI;
        return 0.045 + (cos(theta) * (0.0322 - 0.015));
    }

private:
    long maxEncoderReading;
    long minEncoderReading;

    double input;
    double output;
    double setpoint;

    Encoder *encoder;
    PID *pid;

    static const int EN_A = 25;
    static const int EN_B = 26;
    static const int DIR_PIN_A = 12;
    static const int DIR_PIN_B = 13;
    static const int PWM_PIN = 14;
    static const int COUNTS_PER_REVOLUTION = 700;
};