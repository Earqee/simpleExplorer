
/* Include standard arduino servo control library */
#include <Servo.h>

#include <LiquidCrystal.h>

//#include <VL53L0X.h>

/*
##################################### 

##################################### 
*/

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
    if(servoAngle>75) {
        servoAngle = servoAngle - 3;
        servo.write(servoAngle);
        delay(45);
        return true;
    }
    return false;
}

/* */
bool DistanceSensor::rotateRight() {
    int servoAngle = servo.read();
    if(servoAngle<105) {
      servoAngle = servoAngle + 3;
      servo.write(servoAngle);
      delay(45);
      return true;
    }
    return false;
}

/*
##################################### 

##################################### 
*/

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


/*
##################################### 

##################################### 
*/

LiquidCrystal liquidCrystal(8, 7, 5, 4, 3, 2);  

class LCD {

public:
     
    
    LCD();
    void setup();
    void print(int);

private:  
     
};

LCD::LCD() {}

void LCD::setup() {
    liquidCrystal.begin(16, 2); 
}

void LCD::print(int distance) {

    //Limpa a tela
    liquidCrystal.clear();
    //Posiciona o cursor na coluna 3, linha 0;
    liquidCrystal.setCursor(1, 0);
    //Envia o texto entre aspas para o LCD
    liquidCrystal.print("Dist. Estim.:");
    liquidCrystal.setCursor(1, 1);
    char buffer[15];
    
    sprintf(buffer, "%d (cm)", distance);

    liquidCrystal.print(buffer);
    delay(250);  
}

/*
##################################### 

##################################### 
*/

//VL53L0X testing;


//DistanceSensor sensor;

UltrasonicSensor ultrasonicSensor;


LCD lcd;

/*

void testSetup() {

    pinMode(12,INPUT_PULLUP);
    digitalWrite(12,HIGH);
    Serial.begin(9600);
    Wire.begin();

    testing.init();
    testing.setTimeout(500);

    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).
    testing.startContinuous();
}

void loopTest(){ 
     //###########################

    int distance =sensor.readRangeContinuousMillimeters();
  //int distance =sensor.startContinuous(100);
  
 //distance = distance;
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.print("mm");
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
           

   //####################################
}

*/

void setup() {
    // put your setup code here, to run once:
    ultrasonicSensor.setup();
    //sensor.setup(9);
    lcd.setup();    

    /* */
    pinMode(6, OUTPUT);
}



void loop() {

    int distance = ultrasonicSensor.measure();       
    lcd.print(distance);

    if(distance < 25) {
      /*Write 0V on ESP port (go reverse) */
      analogWrite(6, 0);  
    } else if(distance < 45) {
      /*Write 1.2V on ESP port (go left or right) */
      analogWrite(6, 60);
    } else {
      /*Write 3.3V on ESP port (go front) */
      analogWrite(6, 160);              
    }
    
}