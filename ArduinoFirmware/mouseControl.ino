#include <Arduino.h>
#include <Servo.h>

void readSerial(void);
void dataToNormal(void);

int data[9];
int value[4];

Servo Stretch;
Servo Rotation;
Servo Height;
Servo HandRot;
Servo Grip;

const int stretchMin = 0;
const int stretchMax = 180;
const int stretchNormal = 160;

const int rotationMin = 0;
const int rotationMax = 180;
const int rotationNormal = 75;

const int heightMin = 0;
const int heightMax = 180;
const int heightNormal = 50;

const int handrotMin = 0;
const int handrotMax = 180;
const int handrotNormal = 130;

const int gripMin = 60;
const int gripMax = 157;
const int gripNormal = 120;

int grip = gripNormal;

void setup(){
    Stretch.attach(10);
    Rotation.attach(8);
    Height.attach(9);
    HandRot.attach(11);
    Grip.attach(12);

    Grip.write(gripNormal);
    Stretch.write(stretchNormal);
    Rotation.write(rotationNormal);
    Height.write(heightNormal);
    HandRot.write(handrotNormal);
    Serial.begin(9600);
}

void loop(){
    readSerial();
    dataToNormal();

    if ( (value[0] <= stretchMax) & (value[0] >= stretchMin) ){
        Stretch.write(value[0]);
    }

    if ( (value[1] <= rotationMax) & (value[1] >= rotationMin) ){
        Rotation.write(value[1]);
    }

    if ( (value[2] <= heightMax) & (value[2] >= heightMin) ){
        Height.write(value[2]);
    }

    if ( (value[3] <= handrotMax) & (value[3] >= handrotMin) ){
        HandRot.write(value[3]);
    }

    if ( (data[8] == 0x01) & (grip <= gripMax) ) {
        Grip.write(grip+=3);
        data[8] = 0;
    }

    if ( (data[8] == 0x10) & (grip >= gripMin) ){
        Grip.write(grip-=3);
        data[8] = 0;
    }

}

void readSerial(){
  while(true){
    if (Serial.available() == 11)                 // one packet contains 11 bytes
            if (Serial.read() == 0xFF)            //
                if (Serial.read() == 0xAA){       //
                    data[0] = Serial.read();      // Flag Rotation 0x00 or 0xFF
                    data[1] = Serial.read();      // Rotation
                    data[2] = Serial.read();      // Stratch flag
                    data[3] = Serial.read();      // Stretch
                    data[4] = Serial.read();      // Flag Height 0x00 or 0xFF
                    data[5] = Serial.read();      // Height
                    data[6] = Serial.read();      // Flag HandRot
                    data[7] = Serial.read();      // HandRot
                    data[8] = Serial.read();
                    data[9] = Serial.read(); 
                    break;    
                }
  }
}

void dataToNormal(){
    // stretch
    value[0] = stretchNormal + data[2] - data[3];
    //rotation
    value[1] = data[0] - data[1] + rotationNormal;
    //height
    value[2] = data[4] - data[5] + heightNormal;
    //handrot
    value[3] = data[6] - data[7] + handrotNormal;
}
