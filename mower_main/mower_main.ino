#include <MeAuriga.h>
#include <SoftwareSerial.h>

#define BUFF_LEN 10
void bluetoothTransmit(byte *arr)
{
  Serial.write(arr, BUFF_LEN);
}

void setup() {
  Serial.begin(115200);
  
}

void loop() {
  byte arr[BUFF_LEN] = {0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69, 0x6a}; // a, b, c, d, e...
  bluetoothTransmit(arr);
  
}
