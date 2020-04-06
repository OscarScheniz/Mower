#include <MeAuriga.h>
#include <SoftwareSerial.h>
#include <Time.h>

//Different modes in statemachine
#define BLUETOOTH_MODE    0x00
#define ULTRASONIC_SENSOR 0x01
#define IR_SENSOR         0x02
#define MANUAL_MODE       0x03
#define AUTONOM_MODE      0x04

#define BUFF_LEN          10

MeUltrasonicSensor *us = NULL; //PORT 10
MeLineFollower line(PORT_9);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderMotor encoders[2];

uint8_t irRead = 0;
int16_t moveSpeed = 180;

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
}

void Backward(){
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void turnLeft(){
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void turnRight(){
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

void Stop(){
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
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
    case S1_IN_S2_IN: //Serial.println("Sensor 1 and 2 are inside of black line"); 
    break;
    case S1_IN_S2_OUT: //Serial.println("Sensor 2 is outside of black line"); 
    break;
    case S1_OUT_S2_IN: //Serial.println("Sensor 1 is outside of black line"); 
    break;
    case S1_OUT_S2_OUT: //Serial.println("Sensor 1 and 2 are outside of black line"); 
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


void bluetoothTransmit(byte *arr)
{
  Serial.write(arr, BUFF_LEN);
}

void setup() {
  Serial.begin(115200);
  Forward();
}

void loop() {
  //byte arr[BUFF_LEN] = {0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a}; // a, b, c, d, e...
  //bluetoothTransmit(arr);
  readSensor(IR_SENSOR);
  readSensor(ULTRASONIC_SENSOR);
  
}
