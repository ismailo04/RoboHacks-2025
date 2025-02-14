#include "L298N.h"

L298N::L298N(byte enableAPin, byte enableBPin, byte input1Pin, byte input2Pin, byte input3Pin, byte input4Pin){
  this->enableAPin = enableAPin;
  this->enableBPin = enableBPin;
  this->input1Pin = input1Pin;
  this->input2Pin = input2Pin;
  this->input3Pin = input3Pin;
  this->input4Pin = input4Pin;
}

void L298N::init() {
  pinMode(enableAPin, OUTPUT);
  pinMode(enableBPin, OUTPUT);
  pinMode(input1Pin, OUTPUT);
  pinMode(input2Pin, OUTPUT);
  pinMode(input3Pin, OUTPUT);
  pinMode(input4Pin, OUTPUT);
}

void L298N::driveMotorAForward(){
  digitalWrite(input1Pin, LOW);
  digitalWrite(input2Pin, HIGH);
}
void L298N::driveMotorBForward(){
  digitalWrite(input3Pin, LOW);
  digitalWrite(input4Pin, HIGH);
}

void L298N::driveMotorABackward() {
  digitalWrite(input1Pin, HIGH);
  digitalWrite(input2Pin, LOW);
}
void L298N::driveMotorBBackward() {
  digitalWrite(input3Pin, HIGH);
  digitalWrite(input4Pin, LOW);
}

void L298N::setMotorASpeed(long speed){
  analogWrite(enableAPin, speed);
}

void L298N::setMotorBSpeed(long speed){
  analogWrite(enableBPin, speed);
}