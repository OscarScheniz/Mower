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
#define COLLISION_DIST    20 

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
MeUltrasonicSensor ultrasensor(PORT_10); //PORT 10
MeLineFollower line(PORT_9);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderMotor encoders[2];
MeBuzzer buzzer;
MeGyro gyro(1,0x69);
millisDelay timer, timer2;

// Global variables 
uint8_t irRead = 0;
int16_t moveSpeed = 127;
bool delayRunning = false;
bool transmitRunning = false;

/*****TRANSMIT (TX) ARRAY*****
  unit A/M cmd posX posY col 
   0    1   2   3    4    5   
*****************************/
byte txArr[BUFF_LEN] = {0};
byte rxArr[BUFF_LEN] = {0};

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
  Encoder_1.setMotorPwm(moveSpeed/4);
  Encoder_2.setMotorPwm(-moveSpeed);
  delay(1000);
}

void backwardAndTurnRight(){
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed/4);
  delay(1000);
}

void ultrasonic_Process(int distance_cm){
  if ((distance_cm < COLLISION_DIST)){
    setCollisionBit(COLLISION_TRUE);
    int randnum = random (10);
    if (randnum < 5){
       backwardAndTurnLeft();
    }
    else if (randnum >= 5){
       backwardAndTurnRight();
    }
  }
  forward();
}

void ir_Process(int value){
  switch(value){
    
    case S1_IN_S2_IN: 
    setCollisionBit(COLLISION_TRUE);
    backwardAndTurnRight();
    break;
    
    case S1_IN_S2_OUT: 
    setCollisionBit(COLLISION_TRUE);
    turnLeft();
    break;
    
    case S1_OUT_S2_IN: 
    setCollisionBit(COLLISION_TRUE);
    turnRight();
    break;
    
    case S1_OUT_S2_OUT: 
    setCollisionBit(COLLISION_FALSE);
    forward();
    break;
  }
}

void readSensor(int device){
  int distance = 0;
  
  switch(device)
  {
    case ULTRASONIC_SENSOR:
    ultrasonic_Process(ultrasensor.distanceCm());
    break;

    case IR_SENSOR:
    int value = line.readSensors();
    ir_Process(value);
    break;
  }
}

void dismantleRX(byte arr[]){ 
  while(Serial.available() > 5)
  {
    for(int i = 0; i < 6; i++)
    {
      arr[i] = Serial.read();
      if(!(arr[0] == 1))// data not from app
      { 
        arr[0] = 0;
        return;
      }
    }
  }
}

void bluetoothTransmit(byte* arr)
{

  if (!transmitRunning){ 
     timer2.start(200);
     transmitRunning = true;
     }
  else if (timer2.justFinished()) {  
     Serial.write(arr, BUFF_LEN);
     transmitRunning = false;
  }    
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

void setModebit(int mode){
  txArr[1] = mode;
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

void setCollisionBit(int i){
  txArr[5] = i;
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
  buzzer.tone(300,100);
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
    setModebit(MANUAL_MODE);
    manualDrive(rxArr[2]);
  }
  else if (rxArr[1] == AUTONOM_MODE) // Check if autonom mode is enable
  { 
    setModebit(AUTONOM_MODE);
    readSensor(IR_SENSOR);
    readSensor(ULTRASONIC_SENSOR);
  } 

  calcAnglePos(); // Calculate area of movement

  bluetoothTransmit(txArr); // Send outgoing data
  setCollisionBit(COLLISION_FALSE); 
}
