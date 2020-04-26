#include <MeAuriga.h>
#include <SoftwareSerial.h>
#include <Time.h>

//Different modes in statemachine
#define BLUETOOTH_MODE    0x00
#define ULTRASONIC_SENSOR 0x01
#define IR_SENSOR         0x02
#define MANUAL_MODE       0x03
#define AUTONOM_MODE      0x04
#define VELOCITY          20    //20 CM/s 

#define BUFF_LEN          6
#define MAX_MILLIS_TO_WAIT 300  

MeUltrasonicSensor *us = NULL; //PORT 10
MeLineFollower line(PORT_9);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderMotor encoders[2];

uint8_t irRead = 0;
int16_t moveSpeed = 127;
long timer = 0;
long prev_timer= 0;
int prev_cmd = 1; //start in forward

byte arr[BUFF_LEN] = {0};
byte rxArr[BUFF_LEN] = {0};

void encoder1_process(void)
{
  if (digitalRead(Encoder_1.getPortB()) == 0){
    Encoder_1.pulsePosMinus(); 
  }
  else
  {
    Encoder_1.pulsePosPlus();
  }
}

void encoder2_process()
{
  if (digitalRead(Encoder_2.getPortB()) == 0){
    Encoder_2.pulsePosMinus(); 
  }
  else
  {
    Encoder_2.pulsePosPlus();
  }
}

void Forward(){
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
  setForwardbit();
}

void Backward(){
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

void BackwardAndTurnLeft(){
  Backward();
  delay(1000);
  turnLeft();
  delay(1000);
  Stop();
}

void BackwardAndTurnRight(){
  Backward();
  delay(1000);
  turnRight();
  delay(1000);
  Stop();
}

void init(byte arr[]){

  arr[0] = 0x00;
  arr[1] = 0x00;
  arr[2] = 0x01;
  arr[3] = 0x7F;
  arr[4] = 0x7F;
  arr[5] = 0x00;

  //timer = millis();
  //attachInterrupt(arr[2], callback_func, CHANGE);
  return;
}

void Ultrasonic_Process(int distance_cm)
{
  if ((distance_cm < 20)){
    setCollision(1);
    //bluetoothTransmit(arr);
    int randnum = random (10);
    Serial.println(randnum);
    if (randnum < 5){
      BackwardAndTurnLeft();
    }
    else if (randnum >= 5){
       BackwardAndTurnRight();
    }
  }
  setCollision(0);
  Forward();
}

void Ir_Process(int value){

  switch(value){
    
    case S1_IN_S2_IN: //
    //Serial.println("Sensor 1 and 2 are inside of black line"); //Backward
    setCollision(1);
    BackwardAndTurnRight();
    setCollision(0);
    break;
    
    case S1_IN_S2_OUT: //
    //Serial.println("Sensor 2 is outside of black line"); //Turn left
    setCollision(1);
    turnLeft();
    setCollision(0);
    break;
    
    case S1_OUT_S2_IN: //
    //Serial.println("Sensor 1 is outside of black line"); //Turn right
    setCollision(1);
    turnRight();
    setCollision(0);
    break;
    
    case S1_OUT_S2_OUT: //
    //Serial.println("Sensor 1 and 2 are outside of black line"); //Forward
    setCollision(1);
    Forward();
    setCollision(0);
    break;
  }
}

void readSensor(int device)
{
  int distance = 0;
  
  switch(device)
  {
    case ULTRASONIC_SENSOR:
    if(us == NULL)
    {
      us = new MeUltrasonicSensor(PORT_10);
    }
    Ultrasonic_Process(us->distanceCm());
    break;

    case IR_SENSOR:
    int value = 0;
    pinMode(line.pin1(),INPUT);
    pinMode(line.pin2(),INPUT);
    value = line.dRead1()*2+line.dRead2();
    Ir_Process(value);
    break;
  }
}


void dismantleRX(byte arr[])
{ 
  while(Serial.available() > 0)
  {
    for(int i = 0; i < 6; i++)
    {
      arr[i] = Serial.read();
      if(!(arr[0] == 1)) //data not from app
      {
        arr[0] = 0;
        return;
      }
    }
  }
}

void bluetoothTransmit(byte *arr)
{
  Serial.write(arr, BUFF_LEN);
  delay(100);
}

void manualDrive(int arr)
{
  
  //Check command
  switch(arr){
    case 0: //Stop
    {
      Stop();
      break;
    }

    case 1: //Forward
    {
      Forward();
      break;
    }

    case 2: //Backward
    {
      Backward();
      break;
    }

    case 3: //Right
    {
      turnRight();
      break;
    }

    case 4: //Left
    {
      turnLeft();
      break;
    }   
  }
}

void setStopbit(){
  arr[2] = 0;
}

void setForwardbit(){
  arr[2] = 1;
}

void setBackwardbit(){
  arr[2] = 2;
}

void setRightbit(){
  arr[2] = 3;
}

void setLeftbit(){
  arr[2] = 4;
}

void setCollision(int i){
  arr[6] = i;
}
/*
void calcMovementDistX(){
  
     int dataPointX = 127 + (VELOCITY * ((timer - prevTimer) * 1000)))/2; 
}

void calcMovementDistY(){
  
     int dataPointY = 127 + (VELOCITY * ((timer - prevTimer) * 1000)))/2; 
}

void callback_func()
{
  calcMovementDist(
}*/

void setup() {
  Serial.begin(115200);
  init(arr);
  Forward();
}

void loop() {

  bluetoothTransmit(arr);
  /*if(prev_cmd != arr[2])
  {
    //Reset timer
  }*/

  //prev_cmd = arr[2];

  
  
  dismantleRX(rxArr);
  if(rxArr[1] == 1) //Manual mode
  {
    manualDrive(rxArr[2]);
  }
  
  readSensor(IR_SENSOR);
  readSensor(ULTRASONIC_SENSOR);
  
}
