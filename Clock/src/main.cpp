#include <Arduino.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <TimerThree.h>
#include <PID_v1.h>

#define MIN_PWM 0
#define MAX_PWM 80

#include <Servo.h>
//
// #define TODEG 57.2958
//
//
// const float r = 50;
// const float R = 70;
// const float offset = 20.5;
// const float arm = 42.6;
// float X = 0;
// float Y = 109;
//
// float ANGLE1 = 0;
// float ANGLE2 = 0;
//
Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;
//
// int potpin1 = A0;
// int potpin2 = A1;
// int feedbackpin1 = A4;
// int feedbackpin2 = A5;
//
// float val1;
// float val2;
// float actual_val1;
// float actual_val2;
//
// int feedback1 = 0;
// int feedback2 = 0;
//
// void GetAngle1();
// void GetAngle2();

void setDirection(int dir);
void setSpeed(int pwm);

Encoder knobLeft(2, 3);
Encoder knobRight(18, 19);
LiquidCrystal_I2C lcd(0x3F,20,4);

const int fanPin = 4;

int time = 300;
int old = 0;
int oldRight = 0;
int newRight = 0;

int sensor = 0;
int value = 0;

double Setpoint = 0, Input, Output = 0;

double Kp = 0.5;
double Ki = 1;
double Kd = 0.5;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);

    pinMode(5, OUTPUT);
    // Timer3.initialize(5000);
    //Timer3.pwm(5, 512);
    knobLeft.write(0);
    knobRight.write(20000);
    Serial.begin(9600);
    lcd.init();
    lcd.backlight();
    pinMode(11, OUTPUT);

    myPID.SetOutputLimits(-255, 255);
    myPID.SetSampleTime(20);
    myPID.SetMode(AUTOMATIC);
    // myPID.SetSampleTime(10);

    // Timer1.initialize(40);
    //analogWrite(4, 100);
    //Timer1.pwm(4, 1500);
    //lcd.print("Hello, world!");
    myservo1.attach(7);
    myservo2.attach(10);
    myservo3.attach(11);
    myservo4.attach(6);
    myservo5.attach(12);
    // myservo2.attach(10);
}

void loop() {

    sensor = analogRead(A0);

    value = analogRead(A1);

    int v1 = analogRead(A1);
    int v2 = analogRead(A2);
    int v3 = analogRead(A3);
    int v4 = analogRead(A4);

    v1 = map(v1, 0, 1023, 0, 180);
    v2 = map(v2, 0, 1023, 0, 180);
    v3 = map(v3, 0, 1023, 0, 180);
    v4 = map(v4, 0, 1023, 0, 180);

    sensor = map(sensor, 0, 1023, -255, 255);
    value = map(value, 0, 1023, -255, 255);

    //Serial.println(knobLeft.read());

    if( millis() - old >= time ) {
        old = millis();
        lcd.clear();

        lcd.print(knobLeft.read());
        lcd.print(" ");
        lcd.print(knobRight.read());
    }

    newRight = knobRight.read();
    if ( oldRight != newRight ) {
        // Timer3.initialize(newRight);
        oldRight = newRight;
    }

    myservo1.write( map(knobLeft.read(), 0, 50, 0, 180));
    myservo2.write(v1);
    myservo3.write(v2);
    myservo4.write(v4);
    myservo5.write(v3);

    //Timer3.pwm(5, map(knobLeft.read(), 0, 255, 0, 1024));

    // Input = value;
    // Setpoint = sensor;
    // myPID.Compute();
    //
    // if ( Output < 0 ) {
    //     setDirection(-1);
    //     setSpeed(Output);
    // }
    // else {
    //     setDirection(1);
    //     setSpeed(Output);
    // }
    //
    // Serial.print(sensor);
    // Serial.print(" ");
    // Serial.print(value);
    // Serial.print(" ");
    // Serial.println(Output);

    //analogWrite(11, knobLeft.read());

    // for (float dutyCycle = 30.0; dutyCycle < 100.0; dutyCycle++) {
    //    Serial.print("PWM Fan, Duty Cycle = ");
    //    Serial.println(dutyCycle);
    //    Timer1.pwm(fanPin, (dutyCycle / 100) * 1023);
    //    delay(500);
    //  }

    // X = analogRead(potpin1);
    // Y = analogRead(potpin2);
    // X = map(X, 0, 1023, 0, 180);
    // Y = map(Y, 0, 1023, 2070, 580);
    // myservo1.write(X);
    // myservo2.writeMicroseconds(Y);
    //
    // Serial.print(Y);
    // Serial.print("\n");


    // X = analogRead(potpin1);
    // Y = analogRead(potpin2);
    //
    // feedback1 = analogRead(feedbackpin1);
    // feedback2 = analogRead(feedbackpin2);
    //
    // X = map(X, 0, 1023, -100, 100);
    // Y = map(Y, 0, 1023, 0, 200);
    //
    // GetAngle1();
    // GetAngle2();
    //
    // val1 = ANGLE1 * TODEG;
    // val2 = 180 - ( ANGLE2 * TODEG );
    //
    // actual_val1 = map(val1, 0, 180, 572, 2200);
    // actual_val2 = map(val2, 0, 180, 580, 2070);
    //
    // if ( !isnan(val1) && !isnan(val2) && 572 <= actual_val1 && actual_val1 <= 2200 && 580 <= actual_val2 && actual_val2 <= 2070 ) {
    //     myservo1.writeMicroseconds(actual_val1);
    //     myservo2.writeMicroseconds(actual_val2);
    // }
    //
    // Serial.print(feedback1);
    // Serial.print(" ");
    // Serial.print(feedback2);
    // Serial.print("\n");

    // Serial.print(X);
    // Serial.print(" ");
    // Serial.print(Y);
    // Serial.print(" ");
    // Serial.print(val1);
    // Serial.print(" ");
    // Serial.print(val2);
    // Serial.print(" ");
    // Serial.print(actual_val1);
    // Serial.print(" ");
    // Serial.print(actual_val2);
    // Serial.print("\n");
    //
    // delay(100);

}

void setDirection(int dir) {
    if ( dir > 0 ) {
        digitalWrite(8, HIGH);
        digitalWrite(9, LOW);
    }
    else {
        digitalWrite(8, LOW);
        digitalWrite(9, HIGH);
    }
}
void setSpeed(int pwm){
    pwm = abs(pwm);

    if ( pwm < map(MIN_PWM, 0, 100, 0, 255) )
        pwm = map(MIN_PWM, 0, 100, 0, 255);
    if ( pwm > map(MAX_PWM, 0, 100, 0, 255) )
        pwm = map(MAX_PWM, 0, 100, 0, 255);

    pwm = map(pwm, 0, 255, 0, 1024);
    Timer3.pwm(5, pwm);
}
//
// void GetAngle1() {
//
//     float dist = sqrt(Y*Y + (offset+X)*(offset+X));
//
//     float v1 = ( r*r + Y*Y + (offset+X)*(offset+X) - (R+arm)*(R+arm) ) / ( 2*r*dist );
//     float v2 = ( offset + X ) / dist;
//
//     ANGLE1 =  acos(v1) + acos(v2);
// }
//
// void GetAngle2() {
//
//     float Cx = 0, Cy = 0;
//
//     float ApX = r * cos(ANGLE1) - offset;    // A'x = Ax - r * cos(ANGLE1)
//     float ApY = r * sin(ANGLE1);             // A'y = r * sin(ANGLE1)
//
//
//     //calculate Cx and Cy
//     float m = (ApY - Y) / (ApX - X);
//     float aux = R * sqrt( 1/(m*m+1) );
//
//     Cx = ApX + aux;
//     Cy = ApY + m*aux;
//
//     //calculate ANGLE2
//     float dist = sqrt(Cy*Cy + (offset-Cx)*(offset-Cx));
//
//     float v1 = ( r*r + Cy*Cy + (offset-Cx)*(offset-Cx) - R*R ) / ( 2*r*dist );
//     float v2 = ( offset - Cx ) / dist;
//
//     ANGLE2 =  acos(v1) + acos(v2);
// }


//
//
// #include <Ramp.h>						              // include library
// ramp myRamp;							                // new ramp object (ramp<unsigned char> by default)
//
// void setup() {
//   Serial.begin(9600);					            // begin Serial communication
//
//   // Serial.print("1000 ");		    //
//   // Serial.println(myRamp.value());			    // accessing start value
//
//   //Serial.print("Strating interpolation"); //
//   myRamp.go(1000, 5000, SINUSOIDAL_INOUT);	                  // start interpolation (value to go to, duration)
// }
//
// void loop() {
//   // Serial.print("1000 ");      //
//   // Serial.println(myRamp.update());        // update() return the actual interpolation value
//   Serial.print("sugi PULA");
//   delay(10);
// }

//
//
//
// /*
//  * Simple demo, should work with any driver board
//  *
//  * Connect STEP, DIR as indicated
//  *
//  * Copyright (C)2015-2017 Laurentiu Badea
//  *
//  * This file may be redistributed under the terms of the MIT license.
//  * A copy of this license has been included with this distribution in the file LICENSE.
//  */
// #include <Arduino.h>
// #include "BasicStepperDriver.h"
//
// // Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
// #define MOTOR_STEPS 200
// #define RPM 100
//
// // Since microstepping is set externally, make sure this matches the selected mode
// // If it doesn't, the motor will move at a different RPM than chosen
// // 1=full step, 2=half step etc.
// #define MICROSTEPS 16
// int potpin1 = A0;
//
// // All the wires needed for full functionality
// #define DIR A1
// #define STEP A0
// #define ENABLE 38
// //Uncomment line to use enable/disable functionality
// //#define SLEEP 13
//
// // Acceleration and deceleration values are always in FULL steps / s^2
// #define MOTOR_ACCEL 3000
// #define MOTOR_DECEL 3000
//
// // 2-wire basic config, microstepping is hardwired on the driver
// BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP);
//
// //Uncomment line to use enable/disable functionality
// //BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);
//
// void setup() {
//     pinMode(ENABLE, OUTPUT);
//     digitalWrite(ENABLE, 0);
//     stepper.begin(RPM, MICROSTEPS);
//     stepper.setSpeedProfile(stepper.LINEAR_SPEED, MOTOR_ACCEL, MOTOR_DECEL);
//     // if using enable/disable on ENABLE pin (active LOW) instead of SLEEP uncomment next line
//     // stepper.setEnableActiveState(LOW);
// }
//
// int r = 0;
//
// void loop() {
//
//     // energize coils - the motor will hold position
//     // stepper.enable();
//
//     /*
//      * Moving motor one full revolution using the degree notation
//      */
//     // stepper.rotate(720);
//     //stepper.startRotate(100);
//
//     /*
//      * Moving motor to original position using steps
//      */
//     //stepper.move(-MOTOR_STEPS*MICROSTEPS);
//
//     // pause and allow the motor to be moved by hand
//     // stepper.disable();
//
//     // delay(1000);
//
//     // stepper.rotate(-180);
//
//     // delay(1000);
// }
