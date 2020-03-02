#include <Arduino.h>
#include <Servo.h>
#include <ArduinoJson.h>
#include <Ramp.h>

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;
Servo myservo5;

int POTPIN1 = A5;
int POTPIN2 = A4;
int POTPIN3 = A3;
int POTPIN4 = A1;
int POTPIN5 = A2;

int v1 = 0;
int v2 = 0;
int v3 = 0;
int v4 = 0;
int v5 = 0;

int val1 = 0;
int val2 = 0;
int val3 = 0;
int val4 = 0;
int val5 = 0;

StaticJsonDocument<100> doc;
ramp myRamp1(0);
ramp myRamp2(0);
ramp myRamp3(0);
ramp myRamp4(0);
ramp myRamp5(0);
int rampTime = 500;

const int ledPin1 = 5;
const int ledPin2 = 4;
const int switchPin1 = 2;
const int switchPin2 = 3;
volatile bool switchEnabled = false;


int n = 26;
int pos = 0;
int keyframes[][5] = {
    // {50, 98, 164 ,32, 10},
    // {50, 98, 164 ,32, 100},
    // {50, 98, 164 ,32, 50},

    // {180, 129, 141, 57, 53},
    // {180, 162, 110, 100, 26},
    // {180, 150, 112, 112, 26},
    // {0, 167, 95, 123, 25},
    // {0, 170, 108, 62, 18},
    // {0, 167, 108, 89, 55},
    // {0, 158, 101, 118, 20},
    // {180, 162, 98, 120, 20},
    // {180, 180, 94, 105, 19},
    // {180, 159, 111, 83, 60}

{180, 155, 120, 81, 58},
{180, 158, 122, 76, 46},
{180, 157, 127, 78, 27},
{180, 161, 118, 87, 22},
{180, 171, 104, 98, 27},
{180, 156, 103, 106, 25},
{180, 147, 105, 115, 25},
{166, 142, 106, 117, 23},
{55, 141, 108, 115, 23},
{0, 159, 101, 98, 23},
{0, 163, 118, 58, 23},
{0, 154, 109, 71, 37},
{0, 152, 109, 71, 64},
{0, 144, 121, 73, 60},
{0, 146, 120, 71, 27},
{0, 143, 122, 85, 20},
{0, 152, 109, 104, 18},
{0, 154, 99, 114, 18},
{0, 153, 100, 115, 17},
{0, 149, 103, 114, 17},
{99, 149, 100, 115, 18},
{180, 151, 100, 115, 19},
{180, 164, 99, 95, 18},
{180, 158, 96, 96, 39},
{180, 155, 111, 87, 58},
{180, 157, 114, 80, 56}

};

int nrctr = 0;
// void OnChange() {
//     int oldvalue = switchEnabled;
//
//     switchEnabled = !digitalRead(interruptPin);
//     digitalWrite(ledPin, switchEnabled);
//
// }

void SimpleMode();
void ReplayMode();
void RecordMode();

void setup() {
  Serial.begin(9600);

  myservo1.attach(6);
  myservo2.attach(7);
  myservo3.attach(8);
  myservo4.attach(9);
  myservo5.attach(10);

  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(switchPin1, INPUT_PULLUP);
  pinMode(switchPin2, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(interruptPin), OnChange, CHANGE);

  myRamp1.go(keyframes[0][0], rampTime, LINEAR);
  myRamp2.go(keyframes[0][1], rampTime, LINEAR);
  myRamp3.go(keyframes[0][2], rampTime, LINEAR);
  myRamp4.go(keyframes[0][3], rampTime, LINEAR);
  myRamp5.go(keyframes[0][4], rampTime, LINEAR);

}

void loop() {
    v1 = analogRead(POTPIN1);
    v2 = analogRead(POTPIN2);
    v3 = analogRead(POTPIN3);
    v4 = analogRead(POTPIN4);
    v5 = analogRead(POTPIN5);

    v1 = map(v1, 0, 1023, 0, 180);
    v2 = map(v2, 0, 1023, 0, 180);
    v3 = map(v3, 0, 1023, 0, 180);
    v4 = map(v4, 0, 1023, 0, 180);
    v5 = map(v5, 0, 1023, 0, 180);

    int sw1 = digitalRead(switchPin1);
    int sw2 = digitalRead(switchPin2);

    if ( sw1 == 1 && sw2 == 1 )
        SimpleMode();
    if ( sw1 == 0 )
        ReplayMode();
    if ( sw2 == 0 )
        RecordMode();

}

void SimpleMode() {
    digitalWrite(ledPin1, 0);
    digitalWrite(ledPin2, 0);

    myservo1.write(v1);
    myservo2.write(v2);
    myservo3.write(v3);
    myservo4.write(v4);
    myservo5.write(v5);

    if ( millis() % 2 == 0 ) {
        Serial.print("1_"); //code
        Serial.print(v1); //value
        Serial.print("_");
        Serial.print(v2); //value
        Serial.print("_");
        Serial.print(v3); //value
        Serial.print("_");
        Serial.print(v4); //value
        Serial.print("_");
        Serial.print(v5); //value
        Serial.print("#"); //end character
        Serial.println();
        // doc["p1"] = v1;
        // doc["p2"] = v2;
        // doc["p3"] = v3;
        // doc["p4"] = v4;
        // doc["p5"] = v5;
        // serializeJson(doc, Serial);
        // Serial.println("");
    }
}

void ReplayMode() {
    digitalWrite(ledPin1, 1);
    digitalWrite(ledPin2, 0);

    val1 = myRamp1.update();
    val2 = myRamp2.update();
    val3 = myRamp3.update();
    val4 = myRamp4.update();
    val5 = myRamp5.update();
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
        Serial.println("");
    }

    if ( myRamp1.isFinished() ) {
        pos++;
        if (pos == n )
            pos = 0;
        //delay(500);
        myRamp1.go(keyframes[pos][0], rampTime, LINEAR);
        myRamp2.go(keyframes[pos][1], rampTime, LINEAR);
        myRamp3.go(keyframes[pos][2], rampTime, LINEAR);
        myRamp4.go(keyframes[pos][3], rampTime, LINEAR);
        myRamp5.go(keyframes[pos][4], rampTime, LINEAR);
    }
}


void RecordMode() {
    digitalWrite(ledPin1, 0);
    digitalWrite(ledPin2, 1);

    myservo1.write(v1);
    myservo2.write(v2);
    myservo3.write(v3);
    myservo4.write(v4);
    myservo5.write(v5);

    if ( millis() % 1000 == 0 ) {
        doc["p1"] = v1;
        doc["p2"] = v2;
        doc["p3"] = v3;
        doc["p4"] = v4;
        doc["p5"] = v5;
        Serial.print(nrctr);
        Serial.print(" ");
        serializeJson(doc, Serial);
        Serial.println("");
        nrctr++;
    }
}
