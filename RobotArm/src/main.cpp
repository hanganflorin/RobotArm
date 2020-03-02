#include <Arduino.h>
#include <Servo.h>
#define DIM 1000 // maximum dimension of the recorded_data vector
#define PERIOD 50 // period between 2 recorded points, in milliseconds

//pin values for switches and leds
#define LED_PIN_RED 47
#define LED_PIN_GREEN 42
#define LED_PIN_YELLOW 46
#define SW_PIN_MODE1 52
#define SW_PIN_MODE2 50
#define SW_PIN_ENABLE 51

//servo variables. One for each joint
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;

//servo values
uint8_t servo_value1 = 0;
uint8_t servo_value2 = 0;
uint8_t servo_value3 = 0;
uint8_t servo_value4 = 0;
uint8_t servo_value5 = 0;

// 0 = normal operation
// 1 = play
// 2 = record
uint8_t mode = 0;
uint8_t last_mode = 0; //Used tho check when the mode changed

//store the enable switch value
uint8_t enable = 0;

//variables for recording and playing
uint8_t recorded_data[DIM][5];
int dimension = 0; //dimension of the recorded data
int position = 0; //current position while playing
int sgn = 1; //this stores the sign for the playback
long long prev_time = 0; //used to calculate when a PERIOD of time passed

void ReadPotArm(); //reads the values from the potentiometers arm
void ReadMode(); //Read the mode from the switch
void ReadEnable(); //Read the enable value from the switch
void SetServos(); //Sets all servos with the values from the pot arm
void SetServosFromData(); // Sets all servos with the values from recorder_data
void PrintData(); // Prints data. Used for debugging
void SaveData(); // Save the current position in the recorder_data vector

void setup() {
    //Set the serial. Used for debugging
    Serial.begin(9600);

    //Initialize all servos with the correct pins
    servo1.attach(12);
    servo2.attach(11);
    servo3.attach(10);
    servo4.attach(9);
    servo5.attach(8);

    //Sets all pin modes for switches and leds
    pinMode(LED_PIN_GREEN, OUTPUT); //green led +
    pinMode(44, OUTPUT); //green led -
    pinMode(LED_PIN_YELLOW, OUTPUT); //yellow led +
    pinMode(48, OUTPUT); //yellow led -
    pinMode(LED_PIN_RED, OUTPUT); //red led +
    pinMode(49, OUTPUT); //red led -
    pinMode(SW_PIN_MODE1, INPUT_PULLUP); //sw mode 1
    pinMode(SW_PIN_MODE2, INPUT_PULLUP); //sw mode 2
    pinMode(SW_PIN_ENABLE, INPUT_PULLUP); //sw en

    //Set the negative pins of the leds to LOW
    digitalWrite(44, LOW);
    digitalWrite(48, LOW);
    digitalWrite(49, LOW);

}

void loop() {
    ReadPotArm();
    ReadMode();
    ReadEnable();
    // PrintData();

    switch (mode) {
        case 0: {
            // Only sets the servos with the values from the pot arm
            SetServos();
            break;
        }
        case 1: {
            SetServos(); // Sets the servos
            //Every PERIOD of time save a new pint in recoreded_data
            if ( millis() - prev_time >= PERIOD ) {
                prev_time = millis();
                SaveData();
            }
            break;
        }
        case 2: {
            //Every PERIOD of time set the servos with the current data from
            //recorded_data
            if ( millis() - prev_time >= PERIOD ) {
                prev_time = millis();
                SetServosFromData();
            }
            break;
        }
    }
}

void ReadPotArm() {
    //Read every value from the analog pins and map it to the correct values
    //Also apply a constrain between 0 and 180 to make sure that the data
    //passed to the servos is a valid angle
    servo_value1 = constrain(map(analogRead(A1), 0, 1023, -10, 210), 0, 180);
    servo_value2 = constrain(map(analogRead(A2), 0, 1023, -10, 210), 0, 180);
    servo_value3 = constrain(map(analogRead(A3), 0, 1023, 210, -10), 0, 180);
    servo_value4 = constrain(map(analogRead(A4), 0, 1023, 180, 0), 0, 180);
    servo_value5 = constrain(map(analogRead(A5), 0, 1023, 0, 180), 0, 180);
}

void ReadMode() {
    //Read the 2 inputs and based on them sets the mode
    //Also turn on the corresponding LEDs
    int v1 = digitalRead(52);
    int v2 = digitalRead(50);
    if ( v1 == 1 && v2 == 1 ) {
        last_mode = mode;
        mode = 0;
        digitalWrite(LED_PIN_GREEN, LOW); //turn off both leds
        digitalWrite(LED_PIN_YELLOW, LOW);
    }
    else if ( v1 == 0 && v2 == 1 ) {
        if ( last_mode != 1 ) { //Reset position and dimension
            position = 0;
            dimension = 0;
        }
        last_mode = mode;
        mode = 1;
        digitalWrite(LED_PIN_GREEN, LOW); //turn on yellow led
        digitalWrite(LED_PIN_YELLOW, HIGH);
    }
    else if ( v1 == 1 && v2 == 0 ) {
        last_mode = mode;
        mode = 2;
        digitalWrite(LED_PIN_GREEN, HIGH); //turn on green led
        digitalWrite(LED_PIN_YELLOW, LOW);
    }
}

void SetServos() {
    //Set all servos with the values from the pot arm
    if ( enable ) {
        servo1.write(servo_value1);
        servo2.write(servo_value2);
        servo3.write(servo_value3);
        servo4.write(servo_value4);
        servo5.write(servo_value5);
    }
}

void SaveData() {
    //Save a new point in recorded_data
    //For every servo save its value
    if ( dimension < DIM ) {
        recorded_data[dimension][0] = servo_value1;
        recorded_data[dimension][1] = servo_value2;
        recorded_data[dimension][2] = servo_value3;
        recorded_data[dimension][3] = servo_value4;
        recorded_data[dimension][4] = servo_value5;
        dimension++;
    }
}

void SetServosFromData() {
    //Set all servos with the current position from recorded_data
    //Position increments to the last recorded data then it decrements to the
    //first one and so one
    if (!enable)
        return;

    //Reset sgn if position passed the bounds
    if ( position >= dimension || position < 0 ) {
        sgn *= -1;
        position += sgn;
    }

    //Write to the servos the current data point
    servo1.write(constrain(recorded_data[position][0], 0, 180));
    servo2.write(constrain(recorded_data[position][1], 0, 180));
    servo3.write(constrain(recorded_data[position][2], 0, 180));
    servo4.write(constrain(recorded_data[position][3], 0, 180));
    servo5.write(constrain(recorded_data[position][4], 0, 180));
    position += sgn;
}

void ReadEnable() {
    //Read the enable value and set the red LED
    enable = digitalRead(SW_PIN_ENABLE) == 0 ? 1 : 0;
    digitalWrite(LED_PIN_RED, enable);
}

void PrintData() {
    //Print data to serial. Used for debugging
    Serial.print(servo_value1);
    Serial.print(" ");
    Serial.print(servo_value2);
    Serial.print(" ");
    Serial.print(servo_value3);
    Serial.print(" ");
    Serial.print(servo_value4);
    Serial.print(" ");
    Serial.print(servo_value5);
    Serial.print("\n");
}
