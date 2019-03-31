#include <Arduino.h>
#include <Servo.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;

int POTPIN1 = A0;
int POTPIN2 = A1;
int POTPIN3 = A2;

int servo_val1 = 90;
int servo_actual_val1 = 90;
int servo_val2 = 90;
int servo_actual_val2 = 90;
int servo_val3 = 90;
int servo_actual_val3 = 90;

void setup() {
  Serial.begin(9600);

  myservo1.attach(9);
  myservo2.attach(10);
  myservo3.attach(11);
}

void loop() {
  servo_val1 = analogRead(POTPIN1);
  servo_val2 = analogRead(POTPIN2);
  servo_val3 = analogRead(POTPIN3);

  servo_val1 = map(servo_val1, 0, 1023, 0, 180);
  servo_val2 = map(servo_val2, 0, 1023, 0, 180);
  servo_val3 = map(servo_val3, 0, 1023, 0, 180);
  //servo_val3 = (servo_val1 + servo_val2) - 90;

  Serial.print(servo_val1);
  Serial.print("hello from the other side");
  Serial.print(servo_val2);
  Serial.print(" ");
  Serial.print(servo_val3);
  Serial.print(" \n");

  servo_actual_val1 = constrain(map(servo_val1, 0, 180, 0, 160), 0, 180);
  servo_actual_val2 = constrain(map(servo_val2, 0, 180, 0, 160), 0, 180);
  servo_actual_val3 = constrain(map(servo_val3, 0, 180, 0, 160), 0, 180);

  myservo1.write(servo_actual_val1);
  myservo2.write(servo_actual_val2);
  myservo3.write(servo_actual_val3);
}
