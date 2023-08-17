#include <Servo.h>

int topServoPin = 9;
int bottomServoPin = 8;

Servo topServo;
Servo bottomServo;

String servoInByte;
String posInByte;

int servoNum;
int posNum;

void setup() {
  topServo.attach(topServoPin);
  bottomServo.attach(bottomServoPin);
  Serial.begin(9600);
}

void loop() {
  if(Serial.available()){
    servoInByte = Serial.readStringUntil(',');
    servoNum = servoInByte.toInt();
    Serial.read();
    posInByte = Serial.readStringUntil('\n');
    posNum = posInByte.toInt();

    if(servoNum == 1){
      topServo.write(posNum);
      Serial.print("Top Servo in position: ");
    }
    else if(servoNum == 2){
      bottomServo.write(posNum);
      Serial.print("Bottom Servo in position: ");
    }
    Serial.print(posNum);
    Serial.print('\n');
  }
}
