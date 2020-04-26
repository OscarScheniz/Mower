#include <MeAuriga.h>
#include <SoftwareSerial.h>
#include <Time.h>

//Different modes in statemachine
#define BLUETOOTH_MODE    0x00
#define ULTRASONIC_SENSOR 0x01
#define IR_SENSOR         0x02
#define MANUAL_MODE       0x03
#define AUTONOM_MODE      0x04

#define BUFF_LEN          6
#define MAX_MILLIS_TO_WAIT 300  

MeUltrasonicSensor *us = NULL; //PORT 10
MeLineFollower line(PORT_9);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderMotor encoders[2];

uint8_t irRead = 0;
int16_t moveSpeed = 180;

byte arr[BUFF_LEN] = {0x00, 0x00, 0x00, 0x7F, 0x7F, 0x00};
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

void Ultrasonic_Process(int distance_cm)
{
  if ((distance_cm < 20)){
    arr[5] = 1; //Set collision true
    bluetoothTransmit(arr);
    int randnum = random (10);
    Serial.println(randnum);
    if (randnum < 5){
      BackwardAndTurnLeft();
    }
    else if (randnum >= 5){
       BackwardAndTurnRight();
    }
  }
   Forward();
}

void Ir_Process(int value){

  switch(value){
    
    case S1_IN_S2_IN: //
    //Serial.println("Sensor 1 and 2 are inside of black line"); //Backward
    BackwardAndTurnRight();
    break;
    
    case S1_IN_S2_OUT: //
    //Serial.println("Sensor 2 is outside of black line"); //Turn left
    turnLeft();
    break;
    
    case S1_OUT_S2_IN: //
    //Serial.println("Sensor 1 is outside of black line"); //Turn right
    turnRight();
    break;
    
    case S1_OUT_S2_OUT: //
    //Serial.println("Sensor 1 and 2 are outside of black line"); //Forward
    Forward();
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
  delay(1000);
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

void setup() {
  Serial.begin(115200);
  Forward();
}

void loop() {
    
  bluetoothTransmit(arr);
  
  dismantleRX(rxArr);
  if(rxArr[1] == 1) //Manual mode
  {
    manualDrive(rxArr[2]);
  }
  
  readSensor(IR_SENSOR);
  readSensor(ULTRASONIC_SENSOR);
  
}
