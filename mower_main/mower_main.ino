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
#define MAX_MILLIS_TO_WAIT 300  

MeUltrasonicSensor *us = NULL; //PORT 10
MeLineFollower line(PORT_9);
MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeEncoderMotor encoders[2];

uint8_t drive_mode = AUTONOM_MODE; // default mode
uint8_t irRead = 0;
int16_t moveSpeed = 180;

   /*****************TRANSMIT ARRAY********************
      start unit A/M cmd posX posY col colX colY end
      0     1    2   3   4    5    6   7    8    9
  *****************************************************/
byte arr[BUFF_LEN] = {0x02, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x03}; // a, b, c, d, e...
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
    
    case S1_IN_S2_IN: //
    Serial.println("Sensor 1 and 2 are inside of black line"); //Continue
    Forward();
    break;
    
    case S1_IN_S2_OUT: //
    Serial.println("Sensor 2 is outside of black line"); //Turn left
    turnLeft();
    break;
    
    case S1_OUT_S2_IN: //
    Serial.println("Sensor 1 is outside of black line"); //Turn right
    turnRight();
    break;
    
    case S1_OUT_S2_OUT: //
    Serial.println("Sensor 1 and 2 are outside of black line"); //Backward
    BackwardAndTurnRight();
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
  unsigned long starttime;
  starttime = millis();
  Serial.println();
  while ( (Serial.available()<BUFF_LEN) && ((millis() - starttime) < MAX_MILLIS_TO_WAIT) ){}      
  // hang in this loop until we either get 10 bytes of data or 0.3 second has gone by
  
  if(Serial.available() < BUFF_LEN)
  {
     // the 10 bytes of data didn't come in - handle the problem
     //Serial.println("ERROR - Didn't get 10 bytes of data!");
  }
  else
  {
    Serial.println("Recieved all bytes"); 
    for(int i = 0; i < BUFF_LEN; i++){
      arr[i] = Serial.read();
    } 
    Serial.flush();
  }  
}

void bluetoothTransmit(byte *arr)
{
  Serial.write(arr, BUFF_LEN);
}

void manualDrive(){

  
}

void initialize(){
  
  drive_mode = rxArr[0]; // vilken position kommer denna data?
  if (drive_mode == AUTONOM_MODE){
    Forward();
  }
  else if (drive_mode == MANUAL_MODE)
    manualDrive();
}

void setup() {
  Serial.begin(115200);
  initialize();
}

void loop() {
  
  bluetoothTransmit(arr);
  dismantleRX(rxArr);
  if( rxArr != 0){
    for(int i = 0; i<BUFF_LEN; i++)
      Serial.println(rxArr[i], HEX);
  }
  
  //readSensor(IR_SENSOR);
  //readSensor(ULTRASONIC_SENSOR);
  
}
