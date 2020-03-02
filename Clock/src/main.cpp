#include <Arduino.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <TimerThree.h>
#include <PID_v1.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Ramp.h>

#define MIN_PWM 0
#define MAX_PWM 80

#include <Servo.h>




//Gyro Variables
float elapsedTime, ttime, timePrev;            //Variables for time control
int gyro_error=0;                             //We use this variable to only calculate once the gyro data error
float Gyr_rawX, Gyr_rawY, Gyr_rawZ;           //Here we store the raw data read
float Gyro_angle_x, Gyro_angle_y;             //Here we store the angle value obtained with Gyro data
float Gyro_raw_error_x, Gyro_raw_error_y;     //Here we store the initial gyro data error

//Acc Variables
int acc_error=0;                              //We use this variable to only calculate once the Acc data error
float rad_to_deg = 180/3.141592654;           //This value is for pasing from radians to degrees values
float Acc_rawX, Acc_rawY, Acc_rawZ;           //Here we store the raw data read
float Acc_angle_x, Acc_angle_y;               //Here we store the angle value obtained with Acc data
float Acc_angle_error_x, Acc_angle_error_y;   //Here we store the initial Acc data error

float Total_angle_x, Total_angle_y;           //Here we store the final total angle

//More variables for the code..
int i;
int mot_activated=0;
long activate_count=0;
long des_activate_count=0;

//////////////////////////////PID FOR ROLL///////////////////////////
float roll_PID, pwm_L_F, pwm_L_B, pwm_R_F, pwm_R_B, roll_error, roll_previous_error;
float roll_pid_p=0;
float roll_pid_i=0;
float roll_pid_d=0;
///////////////////////////////ROLL PID CONSTANTS////////////////////
double roll_kp=4;//3.55
double roll_ki=0.1;//0.003
double roll_kd=0.4;//2.05
float roll_desired_angle = 0;     //This is the angle in which we whant the

//////////////////////////////PID FOR PITCH//////////////////////////
float pitch_PID, pitch_error, pitch_previous_error;
float pitch_pid_p=0;
float pitch_pid_i=0;
float pitch_pid_d=0;
///////////////////////////////PITCH PID CONSTANTS///////////////////
double pitch_kp=0.1;//3.55
double pitch_ki=0.1;//0.003
double pitch_kd=0.1;//2.05
//
// double pitch_kp=1;//3.55
// double pitch_ki=0.003;
// double pitch_kd=2.05

float pitch_desired_angle = 0;     //This is the angle in which we want the gimbal to stay (for now it will be 0) Joystick for future versions

float PWM_pitch, PWM_roll;











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

Encoder knobLeft(3, 2);
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

StaticJsonDocument<100> doc;
ramp myRamp1(159);
ramp myRamp2(98);
ramp myRamp3(164);
ramp myRamp4(32);
ramp myRamp5(13);
int rampTime = 3000;

const int ledPin = 5;
const int interruptPin = 3;
volatile bool switchEnabled = false;

int pos = 0;

void OnChange() {
    int oldvalue = switchEnabled;

    switchEnabled = !digitalRead(interruptPin);
    digitalWrite(ledPin, switchEnabled);

    // if (oldvalue) {
    //     myRamp1.go(158, rampTime, LINEAR);
    //     myRamp2.go(157, rampTime, LINEAR);
    //     myRamp3.go(82, rampTime, LINEAR);
    //     myRamp4.go(127, rampTime, LINEAR);
    //     myRamp5.go(95, rampTime, LINEAR);
    // }
    // else {
    //     myRamp1.go(159, rampTime, LINEAR);
    //     myRamp2.go(98, rampTime, LINEAR);
    //     myRamp3.go(164, rampTime, LINEAR);
    //     myRamp4.go(32, rampTime, LINEAR);
    //     myRamp5.go(13, rampTime, LINEAR);
    // }

}

void setup() {

    pinMode(ledPin, OUTPUT);
    pinMode(interruptPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(interruptPin), OnChange, CHANGE);

    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);

    // pinMode(5, OUTPUT);
    // Timer3.initialize(5000);
    //Timer3.pwm(5, 512);
    knobLeft.write(0);
    knobRight.write(20000);
    Serial.begin(115200);
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
    myservo1.attach(6);
    myservo2.attach(7);
    myservo3.attach(8);
    myservo4.attach(9);
    myservo5.attach(10);
    // myservo2.attach(10);



  Wire.begin();
  //begin the wire comunication
  Wire.beginTransmission(0x68);           //begin, Send the slave adress (in this case 68)
  Wire.write(0x6B);                       //make the reset (place a 0 into the 6B register)
  Wire.write(0x00);
  Wire.endTransmission(true);             //end the transmission
  //Gyro config
  Wire.beginTransmission(0x68
  );           //begin, Send the slave adress (in this case 68)
  Wire.write(0x1B);                       //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                       //Set the register bits as 00010000 (1000dps full scale)
  Wire.endTransmission(true);             //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(0x68);           //Start communication with the address found during search.
  Wire.write(0x1C);                       //We want to write to the ACCEL_CONFIG register
  Wire.write(0x10);                       //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);

  //Serial.begin(9600);                     //Remember to set this same baud rate to the serial monitor
  ttime = millis();                        //Start counting time in milliseconds

  myservo1.write(159);
  myservo2.write(98);
  myservo3.write(164);
  myservo4.write(32);
  myservo5.write(13);
  delay(6000);
  myRamp1.go(158, rampTime, LINEAR);
  myRamp2.go(157, rampTime, LINEAR);
  myRamp3.go(82, rampTime, LINEAR);
  myRamp4.go(127, rampTime, LINEAR);
  myRamp5.go(95, rampTime, LINEAR);
}

void loop() {

    timePrev = ttime;  // the previous time is stored before the actual time read
    ttime = millis();  // actual time read
    elapsedTime = (ttime - timePrev) / 1000;


    Wire.beginTransmission(0x68);            //begin, Send the slave adress (in this case 68)
    Wire.write(0x43);                        //First adress of the Gyro data
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true);           //We ask for just 4 registers
    Gyr_rawX=Wire.read()<<8|Wire.read();     //Once again we shif and sum
    Gyr_rawY=Wire.read()<<8|Wire.read();
    /*Now in order to obtain the gyro data in degrees/seconds we have to divide first
    the raw value by 32.8 because that's the value that the datasheet gives us for a 1000dps range*/
    /*---X---*/
    Gyr_rawX = (Gyr_rawX/32.8);
    /*---Y---*/
    Gyr_rawY = (Gyr_rawY/32.8);
    /*Now we integrate the raw value in degrees per seconds in order to obtain the angle
    * If you multiply degrees/seconds by seconds you obtain degrees */
      /*---X---*/
    Gyro_angle_x = Gyr_rawX*elapsedTime;
    /*---X---*/
    Gyro_angle_y = Gyr_rawY*elapsedTime;




    //////////////////////////////////////Acc read/////////////////////////////////////
    Wire.beginTransmission(0x68);     //begin, Send the slave adress (in this case 68)
    Wire.write(0x3B);                 //Ask for the 0x3B register- correspond to AcX
    Wire.endTransmission(false);      //keep the transmission and next
    Wire.requestFrom(0x68,6,true);    //We ask for next 6 registers starting withj the 3B
    /*We have asked for the 0x3B register. The IMU will send a brust of register.
    * The amount of register to read is specify in the requestFrom function.
    * In this case we request 6 registers. Each value of acceleration is made out of
    * two 8bits registers, low values and high values. For that we request the 6 of them
    * and just make then sum of each pair. For that we shift to the left the high values
    * register (<<) and make an or (|) operation to add the low values.
    If we read the datasheet, for a range of+-8g, we have to divide the raw values by 4096*/
    Acc_rawX=(Wire.read()<<8|Wire.read())/4096.0 ; //each value needs two registres
    Acc_rawY=(Wire.read()<<8|Wire.read())/4096.0 ;
    Acc_rawZ=(Wire.read()<<8|Wire.read())/4096.0 ;
   /*Now in order to obtain the Acc angles we use euler formula with acceleration values
   after that we substract the error value found before*/
   /*---X---*/
   Acc_angle_x = (atan((Acc_rawY)/sqrt(pow((Acc_rawX),2) + pow((Acc_rawZ),2)))*rad_to_deg) ;
   /*---Y---*/
   Acc_angle_y = (atan(-1*(Acc_rawX)/sqrt(pow((Acc_rawY),2) + pow((Acc_rawZ),2)))*rad_to_deg) ;


   //////////////////////////////////////Total angle and filter/////////////////////////////////////
   /*---X axis angle---*/
   Total_angle_x = 0.98 *(Total_angle_x + Gyro_angle_x) + 0.02*Acc_angle_x;
   /*---Y axis angle---*/
   Total_angle_y = 0.98 *(Total_angle_y + Gyro_angle_y) + 0.02*Acc_angle_y;


   // Serial.print("GyroX angle: ");
   // Serial.print(90 - Total_angle_x);
   // Serial.print("   |   ");
   // Serial.print("GyroY angle: ");
   // Serial.println(Total_angle_y);

   /*///////////////////////////P I D///////////////////////////////////*/
    roll_desired_angle = 0;   //The angle we want the gimbal to stay is 0 and 0 for both axis for now...
    pitch_desired_angle = 0;

    /*First calculate the error between the desired angle and
    *the real measured angle*/
    roll_error = Total_angle_y - roll_desired_angle;
    pitch_error = Total_angle_x - pitch_desired_angle;
    /*Next the proportional value of the PID is just a proportional constant
    *multiplied by the error*/
    roll_pid_p = roll_kp*roll_error;
    pitch_pid_p = pitch_kp*pitch_error;
    /*The integral part should only act if we are close to the
    desired position but we want to fine tune the error. That's
    why I've made a if operation for an error between -2 and 2 degree.
    To integrate we just sum the previous integral value with the
    error multiplied by  the integral constant. This will integrate (increase)
    the value each loop till we reach the 0 point*/
    if(-3 < roll_error <3)
    {
      roll_pid_i = roll_pid_i+(roll_ki*roll_error);
    }
    if(-3 < pitch_error < 3)
    {
      pitch_pid_i = pitch_pid_i+(pitch_ki*pitch_error);
    }
    /*The last part is the derivate. The derivate acts upon the speed of the error.
    As we know the speed is the amount of error that produced in a certain amount of
    time divided by that time. For taht we will use a variable called previous_error.
    We substract that value from the actual error and divide all by the elapsed time.
    Finnaly we multiply the result by the derivate constant*/
    roll_pid_d = roll_kd*((roll_error - roll_previous_error)/elapsedTime);
    pitch_pid_d = pitch_kd*((pitch_error - pitch_previous_error)/elapsedTime);
    /*The final PID values is the sum of each of this 3 parts*/
    roll_PID = roll_pid_p + roll_pid_i + roll_pid_d ;
    pitch_PID = pitch_pid_p + pitch_pid_i + pitch_pid_d ;
    /*We know taht the min value of PWM signal is -90 (usingservo.write) and the max is 90. So that
    tells us that the PID value can/s oscilate more than -90 and 90 so we constrain those values below*/
    if(roll_PID < -90){roll_PID = -90;}
    if(roll_PID > 90) {roll_PID = 90; }
    if(pitch_PID < -90){pitch_PID = -90;}
    if(pitch_PID > 90) {pitch_PID = 90;}

    roll_previous_error = roll_error;     //Remember to store the previous error.
    pitch_previous_error = pitch_error;   //Remember to store the previous error.

    PWM_pitch = 90 + pitch_PID;           //Angle for each motor is 90 plus/minus the PID value
    PWM_roll = 90 - roll_PID;

    //myservo2.write(PWM_pitch);               //Finally we write the angle to the servos
    //roll.write(PWM_roll);
    //Serial.println(PWM_pitch);


    sensor = analogRead(A0);

    value = analogRead(A1);

    int v1 = analogRead(A1);
    int v2 = analogRead(A2);
    int v3 = analogRead(A3);
    int v4 = analogRead(A4);
    int v5 = analogRead(A5);


    v1 = map(v1, 0, 1023, 0, 180);
    v2 = map(v2, 0, 1023, 0, 180);
    v3 = map(v3, 0, 1023, 0, 180);
    v4 = map(v4, 0, 1023, 0, 180);
    v5 = map(v5, 0, 1023, 0, 180);


    // if ( millis() % 2 == 0 ) {
    //     doc["p1"] = v5;
    //     doc["p2"] = v4;
    //     doc["p3"] = v3;
    //     doc["p4"] = v1;
    //     doc["p5"] = v2;
    //     // serializeJson(doc, Serial);
    //     // Serial.println(v2);
    // }

    // Serial.println(v2);

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



    // myservo1.write( map(knobLeft.read(), 0, 50, 0, 180));
    if ( !switchEnabled ) {
        myservo1.write(v5);
        myservo2.write(v4);
         // myservo2.write( map(knobLeft.read(), 0, 50, 0, 180));
        //myservo2.write(90 - Total_angle_x);
        myservo3.write(v3);
        myservo4.write(v1);
        myservo5.write(v2);

        if ( millis() % 2 == 0 ) {
            doc["p1"] = v5;
            doc["p2"] = v4;
            doc["p3"] = v3;
            doc["p4"] = v1;
            doc["p5"] = v2;
            serializeJson(doc, Serial);
            // Serial.println(v2);
        }
    }
    else
    {
        int val1 = myRamp1.update();
        int val2 = myRamp2.update();
        int val3 = myRamp3.update();
        int val4 = myRamp4.update();
        int val5 = myRamp5.update();
        myservo1.write(val1);
        myservo2.write(val2);
        myservo3.write(val3);
        myservo4.write(val4);
        myservo5.write(val5);

        if ( millis() % 2 == 0 ) {
            doc["p1"] = val1;
            doc["p2"] = val2;
            doc["p3"] = val3;
            doc["p4"] = val4;
            doc["p5"] = val5;
            serializeJson(doc, Serial);
            // Serial.println(v2);
        }

        if ( myRamp1.isFinished() ) {
            if (pos) {
                myRamp1.go(158, rampTime, LINEAR);
                myRamp2.go(157, rampTime, LINEAR);
                myRamp3.go(82, rampTime, LINEAR);
                myRamp4.go(127, rampTime, LINEAR);
                myRamp5.go(95, rampTime, LINEAR);
                pos = 0;
            }
            else {
                myRamp1.go(159, rampTime, LINEAR);
                myRamp2.go(98, rampTime, LINEAR);
                myRamp3.go(164, rampTime, LINEAR);
                myRamp4.go(32, rampTime, LINEAR);
                myRamp5.go(13, rampTime, LINEAR);
                pos = 1;
            }
        }
    }



    // if ( myRamp2.isFinished() )
    //     Serial.println("ramp2 finished");
    // if ( myRamp3.isFinished() )
    //     Serial.println("ramp3 finished");
    // if ( myRamp4.isFinished() )
    //     Serial.println("ramp4 finished");
    // if ( myRamp5.isFinished() )
    //     Serial.println("ramp5 finished");
    // Serial.println(val);

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
//   myRamp.go(1000, 5000, LINEAR);	                  // start interpolation (value to go to, duration)
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
