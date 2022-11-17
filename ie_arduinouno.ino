
/* Include standard arduino servo control library */
#include <Servo.h>

class DistanceSensor {
public:
    DistanceSensor();
    void setup(int);
    bool rotateLeft();
    bool rotateRight();
private:
    Servo servo;
    int sensorRotation;

};

/* */
DistanceSensor::DistanceSensor() {}

/* */
void DistanceSensor::setup(int pin) {
    servo.attach(pin);
}

/* */
bool DistanceSensor::rotateLeft() {
    int servoAngle = servo.read();
    if(servoAngle>0) {
        servoAngle--;
        servo.write(servoAngle);
        delay(45);
        return true;
    }
    return false;
}

/* */
bool DistanceSensor::rotateRight() {
    int servoAngle = servo.read();
    if(servoAngle<180) {
      servoAngle++;
      servo.write(servoAngle);
      delay(45);
      return true;
    }
    return false;
}


class UltrasonicSensor {

public:
    UltrasonicSensor();
    void setup();
    void loop();
        
private:
    /* */
    int soundWaveTravelTime;
    int distanceMeasured;
    
    /* */
    int triggerPin = 11;
    int echoPin = 10;
    
};

UltrasonicSensor::UltrasonicSensor() {}

void UltrasonicSensor::setup() {
    /* */
    pinMode(triggerPin, OUTPUT);
    /* */
    pinMode(echoPin, OUTPUT);   
    /* */    
    Serial.begin(9600);
    Serial.println("Ultrasonic Sensor HC-SR04 Test"); // print some text in Serial Monitor
    Serial.println("with Arduino UNO R3"); 
}

void UltrasonicSensor::loop() {
    /* Start pulse on state low */
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    
    /* Set pulse on state high */
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);

    /* Set pulse on state low */
    digitalWrite(triggerPin, LOW);
    
    /* Reads the echoPin, returns the sound wave travel time in microseconds */
    soundWaveTravelTime = pulseIn(echoPin, HIGH);
    
    /* traveltime of sound wave divided by 2 (go and back) */
    distanceMeasured = soundWaveTravelTime * 0.034 / 2; 

    /* Displays the distance on the Serial Monitor */  
    
    Serial.print("Distance: ");
    Serial.print(distanceMeasured);
    Serial.println(" cm");
}

DistanceSensor sensor;

UltrasonicSensor ultrasonicSensor;

void setup() {
    // put your setup code here, to run once:
    //Movement movement;
    //ultrasonicSensor.setup();
}

void loop() {
    // put your main code here, to run repeatedly:
    //Movement movement;
    //movement.moveLeft();
    //DistanceSensor sensor;
    //while(sensor.rotateLeft());
    //while(sensor.rotateRight());
    //ultrasonicSensor.loop();
}