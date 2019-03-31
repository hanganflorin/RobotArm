#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Servo.h>
#include <PID_v1.h>

double Setpoint = 90, Input, Output;
//Define the aggressive and conservative Tuning Parameters
// double aggKp=4, aggKi=0.2, aggKd=1;
double Kp=1, Ki=2, Kd=0;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Servo servo;

int POTPIN = A0;

float new_value = 0;
float old_value = 0;
float A = 0.2;
float B = 0.8;
double mapped_value = 0;
int servo_val = 90;
int servo_actual_val = 90;

int desired = 90;
int error = 10;
int increment = 2;

int roll = 0;
int pitch = 0;

void displaySensorDetails(void)
{
    sensor_t sensor;
    accel.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print  ("Sensor:       "); Serial.println(sensor.name);
    Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
    Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
    Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void setup(void)
{
    Serial.begin(9600);
    if(!accel.begin())
    {
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
    }
    accel.setRange(ADXL345_RANGE_8_G);

    servo.attach(9);

    myPID.SetOutputLimits(0, 180);
    myPID.SetMode(AUTOMATIC);

    //displaySensorDetails();
}

void loop(void)
{
    servo_val = analogRead(POTPIN);
    servo_val = map(servo_val, 0, 1023, 0, 180);
    servo_actual_val = constrain(map(servo_val, 0, 180, 0, 160), 0, 180);
    servo.write(servo_actual_val);

    sensors_event_t event;
    accel.getEvent(&event);

    roll = atan2(event.acceleration.y , event.acceleration.z) * 57.3;
    pitch = atan2((- event.acceleration.x) , sqrt(event.acceleration.y * event.acceleration.y + event.acceleration.z * event.acceleration.z)) * 57.3;

    old_value = -pitch + 90;
    new_value = A*old_value + B*new_value;
    //mapped_value = map(new_value, 0, 180, 0, 160);
    Input = new_value;
    myPID.Compute();

    // servo_actual_val = constrain(map(Output, 0, 180, 0, 160), 0, 180);
    // servo.write(servo_actual_val);


    // if ( mapped_value <= desired - error && servo_val < 180 )
    //     servo_val += increment;
    // if ( mapped_value >= desired + error && servo_val > 0)
    //     servo_val -= increment;

    // servo.write(constrain(mapped_value, 0, 180));

    Serial.print(new_value); //de la senzor
    Serial.print(" ");
    Serial.print(servo_val);
    Serial.print("\n");

    //delay(10);
}
