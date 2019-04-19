#include "Arduino.h"
#include <SparkFun_ADXL345.h>

#ifndef adxl345Sensor_h
#define adxl345Sensor_h

#define offsetX   0.01       // OFFSET values
#define offsetY   -0.05
#define offsetZ   0.025

#define gainX     1.11        // GAIN factors
#define gainY     1.1
#define gainZ     1.005

#define ALPHA     0.4  //ALPHA for low pass filter

#define TO_G      0.00390625  //convert raw data to G's
#define TO_DEGREES 57.2957795131 //convert from rad to degress (180/pi)

class adxl345Sensor
{
public:
    adxl345Sensor();
    void init();
    float GetPitch();
private:
    ADXL345 adxl;
    float accX = 0;
    float accY = 0;
    float accZ = 0;

    float pitch = 0;
    float pitchF = 0;

};

#endif
