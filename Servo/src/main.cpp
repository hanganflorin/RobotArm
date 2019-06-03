#include <Arduino.h>
#include <Servo.h>

Servo myservo1;

int potpin1 = A0;
int val = 0;

void setup() {
    // pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    // digitalWrite(4, 1);
    // digitalWrite(5, 1);
    tone(4, 50000);

    myservo1.attach(4, 50, 2000);
}

void loop() {
    // val = analogRead(potpin1);
    // val = map(val, 0, 1023, 0, 100000);
    // tone(4, val);
    // myservo1.write(val);
    // if ( val < 0 ) {
    //     digitalWrite(4, 1);
    //     analogWrite(5, 255-abs(val));
    // }
    // else {
    //     digitalWrite(5, 1);
    //     analogWrite(4, 255-abs(val));
    // }
}
