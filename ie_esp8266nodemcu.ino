
#ifndef Movement_H
#define Movement_H

/* */

class Movement {
public:
    Movement();
    Movement(int, int, int, int, int, int);
    void setup();
    void goFront();
    void goReverse();
    void moveLeft();
    void moveRight();
    void increaseSpeed();
    void decreaseSpeed();

private:
/* Wheel Control */
    int leftWheel = D6;
    int leftWheelReverse = D7;
    int rightWheel = D1;
    int rightWheelReverse = D2;
    
/* PWM Controller */
    int pwmPin = D7;
    int pwmVoltage = 0;
    int pwmMaxVoltage = 1023;
};

/* Constructor */
Movement::Movement() {}

/* Initializes used pin */
Movement::Movement(
      int leftWheel, int leftWheelReverse,
      int rightWheel, int rightWheelReverse, 
      int pwmPin, int pwmMaxVoltage) {
    /* */
    this->leftWheel = leftWheel;
    this->leftWheelReverse = leftWheelReverse;
    this->rightWheel = rightWheel;
    this->rightWheelReverse = rightWheelReverse;
    /* */
    this->pwmPin = pwmPin;   
    this->pwmMaxVoltage  = pwmMaxVoltage;
    /* */
    Movement();
}

/* */
void Movement::setup() {
    /* Set all wheel-connected PINs to OUTPUT */
    pinMode(leftWheel, OUTPUT);
    pinMode(leftWheelReverse, OUTPUT);
    pinMode(rightWheel, OUTPUT);
    pinMode(rightWheelReverse, OUTPUT);
    /* Set max pwm voltage */
    analogWriteRange(pwmMaxVoltage);
}

/* Moves to front using maximum available speed */
void Movement::goFront() {
    digitalWrite(leftWheel, 1);
    digitalWrite(rightWheel, 1);
    digitalWrite(leftWheelReverse, 0);
    digitalWrite(rightWheelReverse, 0);
}

/* Moves to reverse using maximum available speed*/
void Movement::goReverse() {
    digitalWrite(leftWheelReverse, 1);
    digitalWrite(rightWheelReverse, 1);
    digitalWrite(leftWheel, 0);
    digitalWrite(rightWheel, 0);
}

/* */
void Movement::moveLeft() {
    digitalWrite(rightWheel, 1);
    digitalWrite(leftWheelReverse, 1);
    digitalWrite(leftWheel, 0);
    digitalWrite(rightWheelReverse, 0);
}

/* */
void Movement::moveRight() {
    digitalWrite(leftWheel, 1);
    digitalWrite(rightWheelReverse, 1);
    digitalWrite(rightWheel, 0);
    digitalWrite(leftWheelReverse, 0);
}

/* */
void Movement::increaseSpeed() {
    if(pwmVoltage < pwmMaxVoltage)
      pwmVoltage++;
    analogWrite(pwmPin, pwmVoltage);
}

/* */
void Movement::decreaseSpeed() {
    if(pwmVoltage > 0)
      pwmVoltage--;
    analogWrite(pwmPin, pwmVoltage);
}


#endif

void setup() {
  // put your setup code here, to run once:
  Movement movement;
    
    

}

void loop() {
  // put your main code here, to run repeatedly:
  Movement movement;
  movement.moveLeft();
}