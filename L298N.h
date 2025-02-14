#ifndef L298N_H
#define L298N_H

#include <Arduino.h>

class L298N {
  private:
    byte enableAPin;
    byte enableBPin;
    byte input1Pin;
    byte input2Pin;
    byte input3Pin;
    byte input4Pin;
  public:
    L298N() {} // Do not use!
    L298N(byte enableAPin, byte enableBPin, byte input1Pin, byte input2Pin, byte input3Pin, byte input4Pin);

    void init();

    void driveMotorAForward();
    void driveMotorBForward();

    void driveMotorABackward();
    void driveMotorBBackward();

    void setMotorASpeed(long speed);
    void setMotorBSpeed(long speed);
};


#endif