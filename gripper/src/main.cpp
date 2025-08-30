#include <Arduino.h>
#include <AccelStepper.h>
#include <vector>
#include <cmath>
#include <limits>
#include "Clamp.hpp"
#include "PrimaryAxis.hpp"
#include "SecondaryAxis.hpp"
#include "ScanningAxis.hpp"
#include "AS726X.h"

PrimaryAxis primaryAxis;
SecondaryAxis secondaryAxis;
Clamp clamp;
ScanningAxis scanningAxis;
AS726X sensor;

byte GAIN = 2;
byte MEASUREMENT_MODE = 0;

struct Position
{
  double yaw;
  double pitch;
};

struct NIRReading
{
  float r;
  float s;
  float t;
  float u;
  float v;
  float w;
};

struct Sample
{
  Position position;
  NIRReading readings;
  double radius;
};

Sample samplePoint(Position &position)
{
  primaryAxis.setTargetPositionRadians(position.pitch);
  secondaryAxis.setTargetPositionRadians(position.yaw);

  while (primaryAxis.distanceToGo() != 0 || secondaryAxis.distanceToGo() != 0)
  {
    primaryAxis.run();
    secondaryAxis.run();
  }
  scanningAxis.engage();
  delay(500);
  sensor.takeMeasurementsWithBulb();
  const double orangeRadius = scanningAxis.getDistanceToCenter();

  scanningAxis.disengage();
  scanningAxis.stop();

  return {
      .position = Position{.yaw = position.yaw, .pitch = position.pitch},
      .readings = NIRReading{
          .r = sensor.getCalibratedR(),
          .s = sensor.getCalibratedS(),
          .t = sensor.getCalibratedT(),
          .u = sensor.getCalibratedU(),
          .v = sensor.getCalibratedV(),
          .w = sensor.getCalibratedW(),
      },
      .radius = orangeRadius};
}

double positionDistance(const Position &a, const Position &b)
{
  int dyaw = a.yaw - b.yaw;
  int dpitch = a.pitch - b.pitch;
  return sqrt(dyaw * dyaw + dpitch * dpitch);
}

std::vector<Position> generateFibonacciPositions(int numPoints)
{
  std::vector<Position> positions;
  const double goldenAngle = M_PI * (3.0f - sqrt(5.0f));

  for (int i = 0; i < numPoints; i++)
  {
    double y = 1.0f - (i / double(numPoints - 1)) * 2.0f;
    double theta = fmod(goldenAngle * i, 2.0f * M_PI);
    double phi = asin(y);

    positions.push_back({.yaw = theta,
                         .pitch = phi});
  }

  return positions;
}

void runFullScan(std::vector<Sample> &results, int numPoints)
{
  auto points = generateFibonacciPositions(numPoints);
  for (Position &p : points)
  {
    if (p.pitch > radians(60) || p.pitch < -radians(60))
    {
      results.push_back(Sample{
          .position = Position{.yaw = p.yaw, .pitch = p.pitch},
          .readings = NIRReading{
              .r = 0,
              .s = 0,
              .t = 0,
              .u = 0,
              .v = 0,
              .w = 0},
          .radius = 0.0,
      });
      continue;
    }
    results.push_back(samplePoint(p));
  }

  primaryAxis.setTargetPositionRadians(0);
  secondaryAxis.setTargetPositionRadians(0);
  while (primaryAxis.distanceToGo() != 0 || secondaryAxis.distanceToGo() != 0)
  {
    primaryAxis.run();
    secondaryAxis.run();
  }
}

void sendResultsOverSerial(std::vector<Sample> &results)
{
  Serial.println("RESULTS");
  Serial.println("[");
  for (size_t i = 0; i < results.size(); ++i)
  {
    const Sample &sample = results[i];
    if (std::isnan(sample.position.yaw) || std::isinf(sample.radius))
    {
      Serial.println("  {\"error\": \"Invalid sample\"}");
      continue;
    }
    Serial.print("  {\"yaw\":");
    Serial.print(sample.position.yaw, 6);
    Serial.print(", \"pitch\":");
    Serial.print(sample.position.pitch, 6);
    Serial.print(", \"radius\":");
    Serial.print(sample.radius, 4);
    Serial.print(", \"readings\":{");
    Serial.print("\"r\":");
    Serial.print(sample.readings.r, 4);
    Serial.print(", ");
    Serial.print("\"s\":");
    Serial.print(sample.readings.s, 4);
    Serial.print(", ");
    Serial.print("\"t\":");
    Serial.print(sample.readings.t, 4);
    Serial.print(", ");
    Serial.print("\"u\":");
    Serial.print(sample.readings.u, 4);
    Serial.print(", ");
    Serial.print("\"v\":");
    Serial.print(sample.readings.v, 4);
    Serial.print(", ");
    Serial.print("\"w\":");
    Serial.print(sample.readings.w, 4);
    Serial.print("}}");
    if (i < results.size() - 1)
      Serial.println(",");
    else
      Serial.println();
  }
  Serial.println("]");
  Serial.println("END RESULTS");
}

void setup()
{
  Wire.begin();
  sensor.begin(Wire, GAIN, MEASUREMENT_MODE);
  Serial.begin(115200);
  while (!Serial)
    ;
  primaryAxis.setup();

  clamp.setup();
  clamp.unclamp();

  scanningAxis.setup();
  scanningAxis.calibrate();
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
      clamp.clamp();
      Serial.println("DONE");
    }
    else if (input == "unclamp")
    {
      clamp.unclamp();
      Serial.println("DONE");
    }
    if (input == "scan")
    {
      std::vector<Sample> results;
      runFullScan(results, 20);
      Serial.println("DONE");
      sendResultsOverSerial(results);
      results.clear();
    }

    else
    {
      Serial.println("Unknown command");
    }
  }
}
