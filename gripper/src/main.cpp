#include <Arduino.h>
#include <AccelStepper.h>
#include "Clamp.hpp"
#include "PrimaryAxis.hpp"
#include "SecondaryAxis.hpp"

PrimaryAxis primaryAxis;
SecondaryAxis secondaryAxis;
Clamp clamp;

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ;
  clamp.setup();
}

void loop()
{
  if (Serial.available())
  {
    String input = Serial.readStringUntil('\n');
    input.trim();

    Serial.print("Received: ");
    Serial.println(input);

    if (input == "clamp")
    {
      Serial.println("Clamping...");
      clamp.clamp();
    }
    else if (input == "unclamp")
    {
      Serial.println("Unclamping...");
      clamp.unclamp();
    }
    else
    {
      Serial.println("Unknown command");
    }
  }
}
