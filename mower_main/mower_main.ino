#include "MeAuriga.h"
#include "SoftwareSerial.h"
#include "Time.h"
#include "millisDelay.h"
#include <Wire.h>

//Modes in statemachine
#define BLUETOOTH_MODE    0x00
#define ULTRASONIC_SENSOR 0x01
#define IR_SENSOR         0x02

// Constant speed 
#define VELOCITY          20    // 20 cm/s 

// Communication Protocol
#define MOWER             0x00
#define APP               0x01
#define MANUAL_MODE       0x00
#define AUTONOM_MODE      0x01
#define STOP              0x00
#define FORWARD           0x01
#define BACKWARD          0x02
#define RIGHT             0x03
#define LEFT              0x04
#define POSITION_X_START  0x7F
#define POSITION_Y_START  0x7F
#define COLLISION_FALSE   0x00
#define COLLISION_TRUE    0x01

#define BUFF_LEN          6  

// Port setup
MeUltrasonicSensor *us = NULL; //PORT 10
MeLineFollower line(PORT_9);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderMotor encoders[2];
MeBuzzer buzzer;
MeGyro gyro(1,0x69);
millisDelay timer;

// Global variables 
uint8_t irRead = 0;
int16_t moveSpeed = 127;
bool delayRunning = false;

/*****TRANSMIT (TX) ARRAY*****
  unit A/M cmd posX posY col 
   0    1   2   3    4    5   
*****************************/
byte txArr[BUFF_LEN] = {0};
byte rxArr[BUFF_LEN] = {0};

void encoder1_process(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus(); 
  }
  else{
    Encoder_1.pulsePosPlus();
  }
}

void encoder2_process(){
  if (digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus(); 
  }
  else{
    Encoder_2.pulsePosPlus();
  }
}

void forward(){
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
  setForwardbit();
}

void backward(){
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
  setBackwardbit();
}

void turnLeft(){
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
  setLeftbit();
}

void turnRight(){
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
  setRightbit();
}

void Stop(){
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
  setStopbit();
}

void backwardAndTurnLeft(){
  backward();
  delay(1000);
  turnLeft();
  delay(1000);
  Stop();
}

void backwardAndTurnRight(){
  backward();
  delay(1000);
  turnRight();
  delay(1000);
  Stop();
}

void ultrasonic_Process(int distance_cm){
  if ((distance_cm < 20)){
    setCollision(COLLISION_TRUE);
    int randnum = random (10);
    if (randnum < 5){
      backwardAndTurnLeft();
    }
    else if (randnum >= 5){
       backwardAndTurnRight();
    }
  }
  setCollision(COLLISION_FALSE);
  forward();
}

void ir_Process(int value){
  switch(value){
    
    case S1_IN_S2_IN: 
    setCollision(COLLISION_TRUE);
    backwardAndTurnRight();
    setCollision(COLLISION_FALSE);
    break;
    
    case S1_IN_S2_OUT: 
    setCollision(COLLISION_TRUE);
    turnLeft();
    setCollision(COLLISION_FALSE);
    break;
    
    case S1_OUT_S2_IN: 
    setCollision(COLLISION_TRUE);
    turnRight();
    setCollision(COLLISION_FALSE);
    break;
    
    case S1_OUT_S2_OUT: 
    setCollision(COLLISION_TRUE);
    forward();
    setCollision(COLLISION_FALSE);
    break;
  }
}

void readSensor(int device){
  int distance = 0;
  
  switch(device)
  {
    case ULTRASONIC_SENSOR:
    if(us == NULL){
      us = new MeUltrasonicSensor(PORT_10);
    }
    ultrasonic_Process(us->distanceCm());
    break;

    case IR_SENSOR:
    int value = 0;
    pinMode(line.pin1(),INPUT);
    pinMode(line.pin2(),INPUT);
    value = line.dRead1()*2+line.dRead2();
    ir_Process(value);
    break;
  }
}

void dismantleRX(byte arr[]){ 
  while(Serial.available() > 0){
    for(int i = 0; i < 6; i++){
      arr[i] = Serial.read();
      if(!(arr[0] == 1)){ // data not from app
        arr[0] = 0;
        return;
      }
    }
  }
}

void bluetoothTransmit(byte *arr){
  Serial.write(arr, BUFF_LEN);
  delay(100);
}

void manualDrive(int arr){
  switch(arr){
    case STOP:
      Stop();
      break;

    case FORWARD:
      forward();
      break;

    case BACKWARD:
      backward();
      break;

    case RIGHT:
      turnRight();
      break;

    case LEFT:
      turnLeft();
      break;  
  }
}

void setStopbit(){
  txArr[2] = STOP;
}

void setForwardbit(){
  txArr[2] = FORWARD;
}

void setBackwardbit(){
  txArr[2] = BACKWARD;
}

void setRightbit(){
  txArr[2] = RIGHT;
}

void setLeftbit(){
  txArr[2] = LEFT;
}

void setCollision(int i){
  txArr[6] = i;
}

float getAngleZ(){
  gyro.update();
  return gyro.getAngleZ();
}

void calcAnglePos(){
  float angle = getAngleZ();
  int i = 1;
  if(txArr[2] == STOP)
    return;
  if(txArr[2] == BACKWARD){ //In i dimman
    int i = -1; 
  }
  if((angle >= -22) && (angle < 22)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[4] += i;
     delayRunning = false;
   } 
  }
  else if((angle >= 22) && (angle < 66)){
    
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[4]+= i;
     txArr[3]+= i;
     delayRunning = false;
   } 
  }
  else if((angle >= 66) && (angle < 112)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[3]+= i;
     delayRunning = false;
   }   
  }
  else if((angle >= 112) && (angle < 157)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[4]-= i;
     txArr[3]+= i;
     delayRunning = false;
   }   
  }
  else if((angle >= 157) && (angle < 180)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[4]-= i;
     delayRunning = false;
   }  
  }
  else if((angle < -22) && (angle >= -66)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[4]+= i;
     txArr[3]-= i;
     delayRunning = false;
    }    
  }
  else if((angle < -67) && (angle >= -112)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[3]-= i;
     delayRunning = false;
   }  
  }
  else if((angle < -112) && (angle >= -157)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[4]-= i;
     txArr[3]-= i;
     delayRunning = false;
   } 
  }
  else if((angle < -157) && (angle >= -180)){
    if (!delayRunning){ 
     timer.start(1000);
     delayRunning = true;
     }
    else if (timer.justFinished()) {  
     txArr[4]-= i;
     delayRunning = false;
   }    
  }
}

void init(byte txArr[]){

  txArr[0] = MOWER;
  txArr[1] = MANUAL_MODE;
  txArr[2] = STOP;
  txArr[3] = POSITION_X_START;
  txArr[4] = POSITION_Y_START;
  txArr[5] = COLLISION_FALSE;
  
  buzzer.setpin(45);
  buzzer.tone(1000,100);
  buzzer.noTone();

  gyro.begin();
  return;
}

void setup() {
  Serial.begin(115200);
  init(txArr);
}

void loop() {
  
  dismantleRX(rxArr); // Teardown incoming data
  
  if(rxArr[1] == MANUAL_MODE) // Check if manual mode is enable
  {
    manualDrive(rxArr[2]);
  }
  else if (rxArr[1] == AUTONOM_MODE) // Check if autonom mode is enable
  { 
    readSensor(IR_SENSOR);
    readSensor(ULTRASONIC_SENSOR);
  } 

  calcAnglePos(); // Calculate area of movement
  
  bluetoothTransmit(txArr); // Send outgoing data 
}
