/*Filawinder Firmware rebuilt by Arnd Struve
 * Hamburg, 26.02.2019 
 * 
 * Original firmware did not work for me, so I decided to rewrite it. Code length was reduced by half.
 * 
 * Commands:
 * 1) hold left and turn pot to set min
 * 2) hold right and turn pot to set max
 * 3) hold center and turn pot to set current position and direction
 * 4) press left and right for more than 0.5s to start QTR calibration
 */

#include "EEPROMex.h"
#include "EEPROMVar.h"
#include <Servo.h>
#include <QTRSensors.h>
#include <PID_v1.h>

//PINS
#define qtrPin0 0 //Top
#define qtrPin1 1
#define qtrPin2 2
#define qtrPin3 3 //Bottom
#define hallPin  7
#define btnMinPin 3
#define btnMaxPin 8
#define btnSetPin 4
#define swAutoPin 2
#define potPin 6
#define motorPin  5
#define servoPin  6
#define ledPin  13

//debug options
bool debugMode = true; //Activated all kinds of debug messages in serial
bool debugMotorSpeed = false; //Current motor speed in manual and auto mode [%]
bool debugGuide = true;
bool debugInputs = false;

//Servo
int servoMinPos = EEPROM.readInt(70); //Right limit for filament guide
int servoMaxPos = EEPROM.readInt(80);  //Left Limit for Filamnet guide
float servoCurrentPos = EEPROM.readInt(90); //read from current servo position
//float servoCurrentPos = 0;
bool servoCurrentDir = bool(EEPROM.readInt(100)); //Direction the guide is moving
float degPerRotation = 1.17; //Guide shift per registered rotation. 1.75mm --> 1.17 deg
int rotStatus = LOW; //HIGH, when hallSensor goes from high to low

//QTR
unsigned int lastRegisteredPos = 0; //store last non-zero line position
unsigned long lastRegisteredTime = millis(); // time when this non zero position was observed
int timeOut = 1000; //milli seconds before sustained zero position is valid
int cutOffMin = 400;
int cutOffMax = 2600;
unsigned int sensorValues[4];

//Motor
double Setpoint = 1500; //The value PID trys to maintain.  The number controls the amount of tension on the spool.
double Input, Output; //PID variables
unsigned long timeToMinimumSpeed = 0;
int minimumMotorSpeed = 25; //does not turn under 25/255
int currentMotorSpeed = 0;
int motorTimeOut = 2500; //tolerance timeframe, before motor turns off completely

int revCount = 0;

QTRSensorsAnalog qtra((unsigned char[]) {qtrPin3, qtrPin2, qtrPin1, qtrPin0}, 4, 4, QTR_NO_EMITTER_PIN); //4 Sensors used, 4 Samples per Sensor, No emitter pin
PID pullPID(&Input, &Output, &Setpoint, 0.0020, 0, 0.001, REVERSE);    //Specify the links and initial tuning parameters
Servo servo;

void setup() {
  pinMode(qtrPin0, INPUT); //Set Inputs and Outputs
  pinMode(qtrPin1, INPUT);
  pinMode(qtrPin2, INPUT);
  pinMode(qtrPin3, INPUT);
  pinMode(hallPin, INPUT);
  pinMode(btnMinPin, INPUT_PULLUP); //Using internal pullups
  pinMode(btnMaxPin, INPUT_PULLUP);
  pinMode(btnSetPin, INPUT_PULLUP);
  pinMode(swAutoPin, INPUT_PULLUP);
  pinMode(potPin, INPUT);
  pinMode(motorPin, OUTPUT);
  pinMode(servoPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  Serial.begin(38400); //Serial for debugging  
  if(debugMode){Serial.println("GUIDE min: " + String(servoMinPos) + " max: " + String(servoMaxPos) + " current: " + String(servoCurrentPos) + " dir: " + String(servoCurrentDir));}
  servo.attach(servoPin); //Init Servo
  //servoCurrentPos = servo.read();
  digitalWrite(motorPin, currentMotorSpeed); //Set Motorspeed to 0
  initQTR(); //Get saved sensor values from EEPROM
  
  pullPID.SetMode(AUTOMATIC); //turn the PID on
  pullPID.SetControllerDirection(REVERSE);
  pullPID.SetOutputLimits(-5, 5);
  //pullPID.SetSampleTime(500);
}

void loop(){
  while(debugInputs){testInputs();}
  
  //Set Servo Minimum: btnMin and not btnMax
  if (!digitalRead(btnMinPin) && digitalRead(btnMaxPin)){
    delay(500);
    while (!digitalRead(btnMinPin) && digitalRead(btnMaxPin)){
      setMinGuidePosition();
    }
  }
  
  //Set Servo Maximum: btnMax and not btnMin
  if (!digitalRead(btnMaxPin) && digitalRead(btnMinPin)){
    delay(500);
    while (!digitalRead(btnMaxPin) && digitalRead(btnMinPin)){
      setMaxGuidePosition();
    }
  }

  //Set current servo position: Set and not btnMin or btnMax  
  while (!digitalRead(btnSetPin) && digitalRead(btnMinPin) && digitalRead(btnMaxPin)){setGuidePosition();}
  
  //QTR Calibration: Min + Max
  if (!digitalRead(btnMaxPin) && !digitalRead(btnMinPin)){
    delay(500);
    if (!digitalRead(btnMaxPin) && !digitalRead(btnMinPin)){ //still pressed?
      calibrateQTR(); //Will read and store min and max values within the next 5s
    }
  }

  //If none of the above is pressed execute guideControl() and motor control (in auto or manual mode)
  guideControl();
  
  //Manual mode: Auto off
  if (digitalRead(swAutoPin)){
    manualControl();
  }
  
  //Auto mode: Auto on
  if (!digitalRead(swAutoPin)){    
    pullControl();
  }
}

void pullControl(){   
  //unsigned int Input = getLinePos();  //Get line position from sensors
  Input = getLinePos();
  if (!pullPID.Compute()){  //Run the PID 
    return;              
  }
  currentMotorSpeed = currentMotorSpeed + (int)(Output+0.5);
  setMotorSpeed(currentMotorSpeed);    //Set the spool speed to the PID result 
  if(debugMotorSpeed){Serial.println("Auto motor speed: " + String(currentMotorSpeed/2.55) + "%");}
}

void guideControl(){
  //Revolution is registered
  if (rotStatus == LOW && !digitalRead(hallPin)){
    rotStatus = HIGH;
    revCount++;
    if(debugGuide){Serial.println("Rev: " + String(revCount));}
    if (servoCurrentPos >= servoMinPos){ //Min Left about 130 (higher value than max)
      servoCurrentDir = HIGH;
    }
    if (servoCurrentPos <= servoMaxPos){
      servoCurrentDir = LOW;
    }
    if (servoCurrentDir) { //If the current direction of the guide is forward
      servoCurrentPos = (servoCurrentPos - degPerRotation); //Move the guide +1.17 degree for 1.75mm filament      
    }
    else{
      servoCurrentPos = (servoCurrentPos + degPerRotation);
    }
    servo.write(int(servoCurrentPos));
    if(debugGuide){
      if(servoCurrentDir){Serial.println("pos: " + String(servoCurrentPos) + " dir: HIGH Limits: " + String(servoMinPos) + "/" + String(servoMaxPos));}
      else{Serial.println("pos: " + String(servoCurrentPos) + " dir: LOW Limits: " + String(servoMinPos) + "/" + String(servoMaxPos));}
    }       
  }
  //reset trigger
  if (rotStatus == HIGH && digitalRead(hallPin)){
    rotStatus = LOW;
  }
  delay(100); //delay for not reading hall sensor "switch" twice
}

void manualControl(){
  int currentMotorSpeed = analogRead(potPin); // reads the value of the potentiometer (value between 0 and 1023)
  currentMotorSpeed = map(currentMotorSpeed, 0, 1023, 0, 255);  // scale it to use it with the servo (value between 0 and 180)
  analogWrite(motorPin, currentMotorSpeed); // sets the servo position according to the scaled value
  if(debugMotorSpeed){Serial.println("Manual motor speed: " + String(currentMotorSpeed/2.55) + "%");} //Motor speed in percent
}

void setMotorSpeed(int motorSpeed){
  //when motor speed is set lower than the minimal speed --> keep minimal speed for some time
  if (motorSpeed < minimumMotorSpeed && (millis() - timeToMinimumSpeed < motorTimeOut)){
    motorSpeed = minimumMotorSpeed;
  }
  else {
    timeToMinimumSpeed = millis();
  }
  if (motorSpeed < 0){
    motorSpeed = 0;
  }
  if (motorSpeed>255){
    motorSpeed=255;
  }  
  analogWrite(motorPin, motorSpeed);    //Set the spool speed to the PID result
}

unsigned int getLinePos(){
  qtra.readCalibrated(sensorValues);              
  unsigned int currentLinePosition = qtra.readLine(sensorValues, QTR_EMITTERS_OFF, 1);
  //Ignore extreme values 0/3000, when the filament is not in the sensor frame for some time
  if (currentLinePosition == 0 || currentLinePosition == 3000){  
    if (millis()-lastRegisteredTime <= timeOut || (cutOffMin < lastRegisteredPos && lastRegisteredPos < cutOffMax)){
      currentLinePosition = lastRegisteredPos;
    }
  }
  //Line within limits
  else{
    lastRegisteredPos = currentLinePosition;
    lastRegisteredTime = millis();
  }
  return currentLinePosition;
}

void calibrateQTR(){
  analogWrite(motorPin, 0);
  Serial.println("Calibrating QTR sensors...");
  qtra.resetCalibration();
  delay(1000);
  digitalWrite(ledPin, HIGH);
  for (int i = 0; i < 300; i++)  // make the calibration take about 7 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2500 us per read (i.e. ~25 ms per call)
  }
  for (int i = 0; i < 4; i++)
  {
    EEPROM.writeInt(i*2, qtra.calibratedMinimumOn[i]);     //Write the minimum values into EEPROM. Each takes 2 slots, 0-1,2-3,4-5,6-7
    EEPROM.writeInt((i*2)+10, qtra.calibratedMaximumOn[i]);     //Write the minimum values into EEPROM. Each takes 2 slots, 10-11,12-13,14-15,16-17
    Serial.println("sensor" + String(i) + " min/max: " +  String(qtra.calibratedMinimumOn[i])+ "/" + String(qtra.calibratedMaximumOn[i]));
  }
  digitalWrite(ledPin, LOW);
  Serial.println("Calibration done.");
  for (int i = 0; i < 5; i++){ //blink 5 times
    delay(300);
    digitalWrite(ledPin, HIGH);
    delay(700);
    digitalWrite(ledPin, LOW);  
  }
}

void initQTR(){
  //Read values from EEPROM
  qtra.calibrate();  
  for (int i = 0; i < 4; i++){
    qtra.calibratedMinimumOn[i] = EEPROM.readInt(i*2);
    qtra.calibratedMaximumOn[i] = EEPROM.readInt((i*2)+10);   
    if(debugMode){Serial.println("sensor" + String(i) + " min/max: " +  String(EEPROM.readInt(i*2)) + "/" + String(EEPROM.readInt(i*2+10)));}   
  }
  if(debugMode){Serial.println("Sensor values restored from EEPROM.");}
}

void setGuidePosition(){
  analogWrite(motorPin, 0);
  int lastPosition = servoCurrentPos;
  servoCurrentPos = potAverage(5); //Make the current position the max limit  
  servo.write(servoCurrentPos); //Move the guide to the knob position
  if (lastPosition > servoCurrentPos){
    servoCurrentDir = HIGH; //Direction Left-Right
  }
  if (lastPosition < servoCurrentPos){
    servoCurrentDir = LOW; //Direction Right-Left
  }   
  if(debugGuide && servoCurrentDir){Serial.println("servoCurrentPos: " + String(servoCurrentPos) + "\tservoCurrentDir: HIGH");}
  if(debugGuide && !servoCurrentDir){Serial.println("servoCurrentPos: " + String(servoCurrentPos) + "\tservoCurrentDir: LOW");}
  if(digitalRead(btnSetPin)){
    EEPROM.writeInt(90, servoCurrentPos); //write to EEPROM when button is released
    EEPROM.writeInt(100, servoCurrentDir); //write to EEPROM when button is released
    if(debugGuide && servoCurrentDir){Serial.println("Writing servoCurrentPos " + String(servoCurrentPos) + " and servoCurrentDir: HIGH to EEPROM");}
    if(debugGuide && !servoCurrentDir){Serial.println("Writing servoCurrentPos " + String(servoCurrentPos) + " and servoCurrentDir: LOW to EEPROM");}
  }  
}

void setMinGuidePosition(){
  analogWrite(motorPin, 0);
  servoMinPos = potAverage(5); //Make the current position the minlimit
  servo.write(servoMinPos); //Move the guide to the knob position
  if(debugGuide){Serial.println("Min Limit: " + String(servoMinPos));}
  if(digitalRead(btnMinPin)){
    EEPROM.writeInt(70, servoMinPos); //write to EEPROM when button is released
    if(debugGuide){Serial.println("Writing servoMinPos " + String(servoMinPos) + " to EEPROM");}
  } 
}

void setMaxGuidePosition(){
  analogWrite(motorPin, 0);
  servoMaxPos = potAverage(5); //Make the current position the max limit  
  servo.write(servoMaxPos); //Move the guide to the knob position
  if(debugGuide){Serial.println("Max Limit: " + String(servoMaxPos));}
  if(digitalRead(btnMaxPin)){
    EEPROM.writeInt(80, servoMaxPos); //write to EEPROM when button is released
    if(debugGuide){Serial.println("Writing servoMaxPos " + String(servoMaxPos) + " to EEPROM");}
  }  
}

int potAverage(int potReadings){
  //sum readings "potReadings"-times and divide by potReadings. Return mapped value.
  int average = 0;
  for (int i = 0; i < potReadings; i++){
    average += analogRead(potPin);
    delay(1); //delay in between reads for stability  
  }
  average = int(average/potReadings);
  average = map(average, 0, 1023, 180, 0);
  return average;  
}

//FOR DEBUGGING
void testInputs(){
  Serial.print("Min: " + String(digitalRead(btnMinPin)) + "\t");
  Serial.print("Max: " + String(digitalRead(btnMaxPin)) + "\t");
  Serial.print("Set: " + String(digitalRead(btnSetPin)) + "\t");
  Serial.print("Auto: " + String(digitalRead(swAutoPin)) + "\t");
  Serial.print("Pot: " + String(analogRead(potPin)) + "\t");
  Serial.print("S0: " + String(analogRead(qtrPin0)) + "\t");
  Serial.print("S1: " + String(analogRead(qtrPin1)) + "\t");
  Serial.print("S2: " + String(analogRead(qtrPin2)) + "\t");
  Serial.print("S3: " + String(analogRead(qtrPin3)) + "\n");
  delay(100);
}
