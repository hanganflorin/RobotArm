#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include "cppQueue.h"

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Queue Q = new Queue(sizeof(float));
float new_value = 0;
float old_value = 0;
float A = 0.1;
float B = 0.9;

void setup(void)
{
  Serial.begin(9600);

  if(!accel.begin())
  {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  accel.setRange(ADXL345_RANGE_16_G);
}

void loop(void)
{
  sensors_event_t event;
  accel.getEvent(&event);

  old_value = event.acceleration.x;
  new_value = A*old_value + B*new_value;

  Serial.print(event.acceleration.x);
  Serial.print(" ");
  // Serial.print(old_value);
  // Serial.print(" ");
  Serial.print(5 + new_value);
  Serial.print("\n");

  //delay(1000);
}
