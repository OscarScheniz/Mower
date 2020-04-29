# Makeblock Mbot

![41015792](https://user-images.githubusercontent.com/32966642/80582064-619bd800-8a0e-11ea-9700-851d085da7d7.png)

Code is written in C++ based on Arduino Mega using Bluetooth Low Energy (BLE) communication. 

## REQUIRMENTS

#M1.1: The Mower shall be capable of running autonomously within a confined area.

#M1.2: The Mower shall be able to avoid collision objects during automous operation.

#M1.3 The Mower shall be able to accept and execute drive commands given by a remote device.

## FUNCTION DESCRIPTION

void forward() Sets the mower to move forward. Speed is set by global variable moveSpeed (Default = 127).

void backward() Sets the mower to move backward. Speed is set by global variable moveSpeed (Default = 127).

void turnLeft() Sets the mower to turn left . Speed is set by global variable moveSpeed (Default = 127).

void turnRight() Sets the mower to turn right. Speed is set by global variable moveSpeed (Default = 127).

void Stop()	Stops the mower.

void backwardAndTurnLeft() Mower move backwards and turn left. Function runs at collision. 

void backwardAndTurnRight() Mower move backwards and turn right. Function runs at collision.

void ultrasonic_Process(int) US sensor data is processed and checks if collision is true, then acts accordingly. Collision is set by global define COLLISION_DIST (Default = 20 cm)

void ir_Process(int) IR sensor data is processed and checks if collision is true, then acts accordingly. 

void readSensor(int) Input data decides which sensor process is called.

void dismantleRX(byte[]) Reads incoming data and inserts data to local buffer.

void bluetoothTransmit(byte*) Sends data.

void manualDrive(int) Input command decides which direction to go.

void setModebit(int) Sets mode bit in local buffer.

void setStopbit(int) Sets stop bit in local buffer.

void setForwardbit(int) Sets forward direction bit in local buffer.

void setBackwardbit(int) Sets backward direction bit in local buffer.

void setRightbit(int) Sets right direction bit in local buffer.

void setLeftbit(int) Sets left direction bit in local buffer.

void setCollisionBit(int) Sets collision bit in local buffer.

float getAngleZ() Gyroscope data on Z axis is collected.

void calcAnglePos() Depending on incoming angle from gyroscope function calculates movement in x and y axis.

void init(byte[]) Initialize the mower

void setup() Initialize UART communication and the mower 

void loop() Mower main program


