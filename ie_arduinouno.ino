
/* Include standard arduino servo control library */
#include <Servo.h>

#include <LiquidCrystal.h>

#include "VL53L0X.h"

#include <Wire.h>


class InfraredSensor {
public:
    InfraredSensor();
    void setup(int);
    bool rotateLeft();
    bool rotateRight();
    int measure();
    void rotate();
private:
    Servo servo;
    int sensorRotation;
    bool preferLeftRotation = true;
    
    /* */
    VL53L0X sensor; 
    int sampleStartTime = millis(); 
    /* Sampling rate 10 Hz */     
    int samplingRate = 100; 

    int rotationStartTime = millis();
    /* Rotation rate 10 Hz */
    int rotationRate = 10;     
};

/* */
InfraredSensor::InfraredSensor() {}

/* */
void InfraredSensor::setup(int pin) {
    servo.attach(pin);

    Serial.begin(9600);
    
    Wire.begin(); 
    /* Setup I2C interface */
    /* Use 400 kHz I2C */
    Wire.setClock(400000); 

    /* Sensor timeout */
    sensor.setTimeout(500); 
    
    if (!sensor.init())//try to initilise the sensor
    {
        /* Sensor does not respond within the timeout time */
        Serial.println("VL53L0X is not responding");
    }
    else
    {
        //SET THE SENSOR TO LONG RANGE MODE
        /* lower the return signal rate limit (default is 0.25 MCPS) */
        sensor.setSignalRateLimit(0.1);
        /* increase laser pulse periods (defaults are 14 and 10 PCLKs) */
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
        /* Set its timing budget in microseconds longer timing budgets will give more accurate measurements */
        sensor.setMeasurementTimingBudget(40000); 
        /* Sets the interval where a measurement can be requested in milliseconds */
        sensor.startContinuous(50); 
    }      
}

/* */
bool InfraredSensor::rotateLeft() {
    int servoAngle = servo.read();
    if(servoAngle> 60) {
        servoAngle = servoAngle - 50;
        servo.write(servoAngle);
        preferLeftRotation = true;
        return true;
    }
    preferLeftRotation = false;
    return false;    
}

/* */
bool InfraredSensor::rotateRight() {
    int servoAngle = servo.read();
    if(servoAngle<120) {
      servoAngle = servoAngle + 50;
      servo.write(servoAngle);
      preferLeftRotation = false;
      return true;
    }
    preferLeftRotation = true;
    return false;
}


int InfraredSensor::measure() {
    int distance = 0;
    if((millis()- sampleStartTime) > samplingRate) {
        distance = sensor.readRangeContinuousMillimeters();
        Serial.println(distance); //Get a reading in millimeters
        sampleStartTime = millis();
    }
    return distance;
}

void InfraredSensor::rotate(){
    if(this->preferLeftRotation) {
      rotateLeft();      
    } else {
      rotateRight();
    }
}


class UltrasonicSensor {

public:
    UltrasonicSensor();
    void setup();
    int measure();
        
private:
        
    /* */
    int triggerPin = 11;
    int echoPin = 10;
    
};

UltrasonicSensor::UltrasonicSensor() {}

void UltrasonicSensor::setup() {
    /* */
    pinMode(triggerPin, OUTPUT);
    /* */
    pinMode(echoPin, INPUT);   
    /* */    
    Serial.begin(9600);
    Serial.println("Ultrasonic Sensor Test");
    Serial.println("with Arduino UNO R3"); 
}

int UltrasonicSensor::measure() {
    /* Start pulse on state low */
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);

    
    /* Set pulse on state high */
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);

    /* Set pulse on state low */
    digitalWrite(triggerPin, LOW);

    /* Reads the echoPin, returns the sound wave travel time in microseconds */
    long soundWaveTravelTime = pulseIn(echoPin, HIGH);
    
    /* traveltime of sound wave divided by 2 (go and back) */
    /* 340 m/s = 0.034 cm/us */
    float distanceMeasured = soundWaveTravelTime * 0.034 / 2.0; 
        
    return distanceMeasured;
}

LiquidCrystal liquidCrystal(8, 7, 5, 4, 3, 2);  


class LCD {

public:
    LCD();
    void setup();
    void print(int, int);

private:  
     
};

LCD::LCD() {}

void LCD::setup() {
    liquidCrystal.begin(16, 2); 
}

void LCD::print(int ultrasonicDistance, int infraredDistance) {

    /* Limpa a tela */
    liquidCrystal.clear();
    /* Posiciona o cursor na coluna 3, linha 0;*/
    liquidCrystal.setCursor(1, 0);
    /* Envia o texto entre aspas para o LCD */
    liquidCrystal.print("Dist. Estim.:");
    liquidCrystal.setCursor(1, 1);
    char buffer[15];
    
    sprintf(buffer, "%d (cm) %d (cm)", ultrasonicDistance/10, infraredDistance);

    liquidCrystal.print(buffer);
    delay(250);  
}

InfraredSensor infraredSensor;

UltrasonicSensor ultrasonicSensor;

LCD lcd;

void setup() {
    // put your setup code here, to run once:
    ultrasonicSensor.setup();
    
    lcd.setup();    
 
    /* */
    pinMode(6, OUTPUT);
    
    //
    infraredSensor.setup(9);   

}

void loop() {

    int ultrasonicMeasure = ultrasonicSensor.measure();       
    
    int infraredMeasure = infraredSensor.measure();

    lcd.print(ultrasonicMeasure, infraredMeasure);

    infraredSensor.rotate();
   
    if(ultrasonicMeasure < 25) {
      /*Write 0V on ESP port (go reverse) */
      analogWrite(6, 0);  
    } else if(ultrasonicMeasure < 45 || (infraredMeasure > 0 && infraredMeasure < 200) ) {
      /*Write 1.2V on ESP port (go left or right) */
      analogWrite(6, 60);
    } else {
      /*Write 3.3V on ESP port (go front) */
      analogWrite(6, 160);              
    }
    
}
