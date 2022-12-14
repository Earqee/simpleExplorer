
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
    int accessPreference();

private:
/* Wheel Control */
    int leftWheel = D6;
    int leftWheelReverse = D7;
    int rightWheel = D1;
    int rightWheelReverse = D2;
    
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
    this->preferLeftRotation = 0;
    Movement();
}

/* */
void Movement::setup() {
    /* Set all wheel-connected PINs to OUTPUT */
    pinMode(leftWheel, OUTPUT);
    pinMode(leftWheelReverse, OUTPUT);
    pinMode(rightWheel, OUTPUT);
    pinMode(rightWheelReverse, OUTPUT);
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


int Movement::accessPreference() {
    this->preferLeftRotation = !this->preferLeftRotation;
    return !this->preferLeftRotation;
}


#endif

Movement movement;

#include <vector>

std::vector<int> previousMeasuresArray;
int currentLocation = 0;
int average = 0;

void insert(int value) {
  int insertedMeasure = value;
  int erasedMeasure = previousMeasuresArray[currentLocation];
  average = average - erasedMeasure/10 + insertedMeasure/10;
  previousMeasuresArray[currentLocation] = insertedMeasure;
  currentLocation = (currentLocation+1) % 10;  
}

void setup() {
  // put your setup code here, to run once:
    movement.setup();

    pinMode(D5, INPUT);

    Serial.begin(9600);

    previousMeasuresArray.resize(10);
}

void loop() {
  // put your main code here, to run repeatedly:

  int currentVoltageMeasure = analogRead(A0);

  insert(currentVoltageMeasure);

  if(average < 100) {
    /*Receiving ~0V on ESP port (go reverse) */
      movement.goReverse(); 
  } else if(average < 400) {
    /* Receiving <~1.2V on ESP psort (go left or right) */
      if(movement.accessPreference() == 0 || 1) {
          movement.moveRight();        
      } else {    
          movement.moveLeft();         
      } 
  } else {
    /*Receiving >1.2V on ESP port (go front) */
      movement.goFront();
  }

  Serial.print("Average Value = ");
  Serial.print(average);
  Serial.print("  ,  Measured = ");
  Serial.println(currentVoltageMeasure);
  
  /*
  if(digitalRead(D5) == LOW) {
    movement.goFront();
  } else {
    movement.moveLeft();
  }
  */
  
}