/*  *****************************************
 *  ADXL345_Calibration
 *  ADXL345 Hook Up Guide Calibration Example
 *
 *  Utilizing Sparkfun's ADXL345 Library
 *  Bildr ADXL345 source file modified to support
 *  both I2C and SPI Communication
 *
 *  E.Robert @ SparkFun Electronics
 *  Created: Jul 13, 2016
 *  Updated: Sep 13, 2016
 *
 *  Development Environment Specifics:
 *  Arduino 1.6.11
 *
 *  Hardware Specifications:
 *  SparkFun ADXL345
 *  Arduino Uno
 *  *****************************************/

#include <SparkFun_ADXL345.h>

/*********** COMMUNICATION SELECTION ***********/
/*    Comment Out The One You Are Not Using    */
// ADXL345 adxl = ADXL345(10);           // USE FOR SPI COMMUNICATION, ADXL345(CS_PIN);
ADXL345 adxl = ADXL345();             // USE FOR I2C COMMUNICATION

/****************** VARIABLES ******************/
/*                                             */
float AccelMinX = 0;
float AccelMaxX = 0;
float AccelMinY = 0;
float AccelMaxY = 0;
float AccelMinZ = 0;
float AccelMaxZ = 0;

float accX = 0;
float accY = 0;
float accZ = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;
float rollF = 0;
float pitchF = 0;
float yawF = 0;

/************** DEFINED VARIABLES **************/
/*                                             */
#define offsetX   0.01       // OFFSET values
#define offsetY   -0.05
#define offsetZ   0.025

#define gainX     1.11        // GAIN factors
#define gainY     1.1
#define gainZ     1.005

#define ALPHA     0.2

/******************** SETUP ********************/
/*          Configure ADXL345 Settings         */
void setup()
{
  Serial.begin(9600);                 // Start the serial terminal
  // Serial.println("SparkFun ADXL345 Accelerometer Breakout Calibration");
  // Serial.println();

  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(2);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                // Configure the device: 4 wire SPI mode = '0' or 3 wire SPI mode = 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library
}

/****************** MAIN CODE ******************/
/*  Accelerometer Readings and Min/Max Values  */
void loop()
{
  // Serial.println("Send any character to display values.");
  // while (!Serial.available()){}       // Waiting for character to be sent to Serial
  // Serial.println();

  // Get the Accelerometer Readings
  int x,y,z;                          // init variables hold results
  float xg,yg,zg;
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store in variables x,y,z
  xg = x * 0.00390625;
  yg = y * 0.00390625;
  zg = z * 0.00390625;


  if(xg < AccelMinX) AccelMinX = xg;
  if(xg > AccelMaxX) AccelMaxX = xg;

  if(yg < AccelMinY) AccelMinY = yg;
  if(yg > AccelMaxY) AccelMaxY = yg;

  if(zg < AccelMinZ) AccelMinZ = zg;
  if(zg > AccelMaxZ) AccelMaxZ = zg;

  // Serial.print("Accel Minimums: "); Serial.print(AccelMinX); Serial.print("  ");Serial.print(AccelMinY); Serial.print("  "); Serial.print(AccelMinZ); Serial.println();
  // Serial.print("Accel Maximums: "); Serial.print(AccelMaxX); Serial.print("  ");Serial.print(AccelMaxY); Serial.print("  "); Serial.print(AccelMaxZ); Serial.println();
  // Serial.println();


  /* Note: Must perform offset and gain calculations prior to seeing updated results
  /  Refer to SparkFun ADXL345 Hook Up Guide: https://learn.sparkfun.com/tutorials/adxl345-hookup-guide
  /  offsetAxis = 0.5 * (Acel+1g + Accel-1g)
  /  gainAxis = 0.5 * ((Acel+1g - Accel-1g)/1g) */

  // UNCOMMENT SECTION TO VIEW NEW VALUES
  accX = (xg - offsetX)/gainX;         // Calculating New Values for X, Y and Z
  accY = (yg - offsetY)/gainY;
  accZ = (zg - offsetZ)/gainZ;

  // Serial.print(accX);
  // Serial.print(" ");
  // Serial.print(accY);
  // Serial.print(" ");
  // Serial.println(accZ);

  roll = atan2(accY , sqrt(accX * accX + accZ * accZ)) * 57.2957795131;
  pitch = 90 + atan2(accX , sqrt(accY * accY + accZ * accZ)) * 57.2957795131;
  yaw = atan2(sqrt(accX * accX + accY * accY), accZ) * 57.2957795131;


  char signx = x < 0 ? '-' : '+';
  char signy = y < 0 ? '-' : '+';
  char signz = z < 0 ? '-' : '+';

  if ( z < 0 )
    pitch = 360 - pitch;

  rollF = ALPHA * roll + (1.00 - ALPHA) * rollF;
  pitchF = ALPHA * pitch + (1.00 - ALPHA) * pitchF;
  yawF = ALPHA * yaw + (1.00 - ALPHA) * yawF;


  // Serial.print(pitch);
  // Serial.print(" ");
  // Serial.print(roll);
  // Serial.print(" ");
  // Serial.print(yaw);
  // Serial.print(" ");
  // Serial.print(signx);
  // Serial.print(" ");
  // Serial.print(signy);
  // Serial.print(" ");
  // Serial.println(signz);
  //Serial.print(" ");

  Serial.print(rollF);
  Serial.print("/");
  Serial.print(pitchF);
  Serial.print("/");
  Serial.println(yawF);



  // Serial.print("New Calibrated Values: "); Serial.print(accX); Serial.print("  "); Serial.print(accY); Serial.print("  "); Serial.print(accZ);
  // Serial.println();
  //
  // while (Serial.available())
  // {
  //   Serial.read();                    // Clear buffer
  // }
}
