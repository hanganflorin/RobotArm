#include "adxl345Sensor.h"
#include <SparkFun_ADXL345.h>

adxl345Sensor::adxl345Sensor() {
    adxl = ADXL345();
}

void adxl345Sensor::init() {
    adxl.powerOn();
    adxl.setRangeSetting(2);
}

float adxl345Sensor::GetPitch() {
    int x,y,z;
    float xg,yg,zg;

    adxl.readAccel(&x, &y, &z);
    xg = x * TO_G;
    yg = y * TO_G;
    zg = z * TO_G;

    //add offsets
    accX = (xg - offsetX)/gainX;
    accY = (yg - offsetY)/gainY;
    accZ = (zg - offsetZ)/gainZ;

    //calculate pitch
    pitch = 90 + atan2(accX , sqrt(accY * accY + accZ * accZ)) * TO_DEGREES;

    //enable rotation greater than 90
    //disable this for pitch < 90 not to jump from 0 to 360
    //if ( z < 0 && pitch > 90)
    if ( z < 0 )
      pitch = 360 - pitch;

    //add low pass filter
    pitchF = ALPHA * pitch + (1.00 - ALPHA) * pitchF;

    return pitchF;
}
