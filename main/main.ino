#include "L298N.h"
#include <Servo.h>

int LEFT_IR_PIN = A4;
int RIGHT_IR_PIN = A5;
L298N l298N(5, 6, 2, 4, 7, 8);
int MOTOR_SPEED = 90;
// Motor A right
// Motor B left

#define S0 A1
#define S1 A2
#define S2 A0
#define S3 A3
#define sensorOut 10

int redMin = 77;
int redMax = 1230;
int blueMin = 78;
int blueMax = 1230;
int greenMin = 76;
int greenMax = 1210;

int redPW = 0;
int greenPW = 0;
int bluePW = 0;

int redValue;
int greenValue;
int blueValue;

long COLOR_CHANGE_DELAY = 500;
long lastColorChange = 0;

int lastColor = -1; // 0 -> Blue, 1 -> Green
Servo myservo;
int servoAngle = 0;
int SERVO_ANGLE_INTERATION = 35;

void initialize() {
  l298N.init();
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // Set frequency scaling to 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  myservo.attach(9); 
  myservo.write(servoAngle);

  Serial.begin(9600);
}

void setup() {
  initialize();
}

void loop() {
  // put your main code here, to run repeatedly:
  //follow_line();
  getValues();
  //printColorValues();
  long currentTime = millis();

  if (currentTime - lastColorChange > COLOR_CHANGE_DELAY) {
    if (isBlue() && lastColor != 0){
      lastColorChange = millis();
      Serial.println("Blue!");
      lastColor = 0;
      wait10sec();
    } else if (isGreen() && lastColor != 1) {
      lastColorChange = millis();
      Serial.println("Green!");
      lastColor = 1;
      dropSeeds(); // TODO
    }
  }

  follow_line();
}

void printColorValues() {
  if (isRed()) {
    Serial.println("Red!");
  }
  else if (isBlue()) {
    Serial.println("Blue!");
  }
  else if (isGreen())  {
    Serial.println("Green!");
  }
}


void follow_line() {
  // Both sensors are on a black line
  if (is_left_on_black() && is_right_on_black()) {
    stop();
  } else if (!is_left_on_black() && !is_right_on_black()) {
    // If both sensors are NOT on a black line
    forward();
  } else if (is_left_on_black() && !is_right_on_black()) {
    // If only the left sensor is on a black line
    left();
  } else if (!is_left_on_black() && is_right_on_black()) {
    // If only the right sensor is on a black line
    right();
  }
}

bool is_left_on_black(){
  return (digitalRead(LEFT_IR_PIN) == HIGH);
}

bool is_right_on_black(){
  return (digitalRead(RIGHT_IR_PIN) == HIGH);
}

void forward() {
  l298N.setMotorASpeed(MOTOR_SPEED);
  l298N.driveMotorAForward();
  l298N.setMotorBSpeed(MOTOR_SPEED);
  l298N.driveMotorBForward();
}

void left() {
  l298N.setMotorASpeed(MOTOR_SPEED);
  l298N.driveMotorAForward();
  l298N.setMotorBSpeed(MOTOR_SPEED);
  l298N.driveMotorBBackward();
}

void right() {
  l298N.setMotorASpeed(MOTOR_SPEED);
  l298N.driveMotorABackward();
  l298N.setMotorBSpeed(MOTOR_SPEED);
  l298N.driveMotorBForward();
}

void stop() {
  l298N.setMotorASpeed(0);
  l298N.setMotorBSpeed(0);
}

int getRedPW() {
      digitalWrite(S2,LOW);
      digitalWrite(S3,LOW);

      int PW;

      PW = pulseIn(sensorOut,LOW);
      return PW;

    }


int getGreenPW() {
      digitalWrite(S2,HIGH);
      digitalWrite(S3,HIGH);

      int PW;

      PW = pulseIn(sensorOut,LOW);
      return PW;

    }

int getBluePW() {
      digitalWrite(S2,LOW);
      digitalWrite(S3,HIGH);

      int PW;

      PW = pulseIn(sensorOut,LOW);
      return PW;

    }

void getValues() {
  int redPW = getRedPW();
  redValue = map(redPW,redMin,redMax,255,0);

  int greenPW = getGreenPW();
  greenValue = map(greenPW,greenMin,greenMax,255,0);

  int bluePW = getBluePW();
  blueValue = map(bluePW,blueMin,blueMax,255,0);

  // Serial.print("Red: ");
  // Serial.print(redValue);
  // Serial.print(" Green: ");
  // Serial.print(greenValue);
  // Serial.print(" Blue: ");
  // Serial.println(blueValue);
}

bool isBlue() {
  return (isAround(redValue, 190, 10) && 
  isAround(greenValue, 244, 10) && 
  isAround(blueValue, 246, 10));
}

bool isGreen() {
  return (isAround(redValue, 181, 10) && 
  isAround(greenValue, 195, 10) && 
  isAround(blueValue, 195, 10));
}

bool isRed() {
  return (isAround(redValue, 244, 10) && 
  isAround(greenValue, 172, 10) && 
  isAround(blueValue, 177, 10));
}

bool isAround(int num, int target, int tol) {
  return abs(target-num) < tol;
}

void wait10sec() {
  stop();
  delay(10000);
  lastColorChange = millis();
}

void dropSeeds() {
  // To be implemented
  stop();
  servoAngle = servoAngle + SERVO_ANGLE_INTERATION;
  myservo.write(servoAngle);
  delay(1000);
  lastColorChange = millis();
}
