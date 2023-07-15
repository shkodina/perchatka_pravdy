#include <SoftwareSerial.h>
#include <Wire.h>

const int MPU_addr = 0x68; // адрес датчика
int16_t data[7];  

SoftwareSerial SSerial (PD2, PD3);

void setup() {
  // put your setup code here, to run once:
  SSerial.begin(19200);

  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  // Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  getData();  // получаем
  // выводим 

  SSerial.print('*');

  uint8_t adata=analogRead(A0);
  // Serial.print(adata);
  // Serial.print(' ');
  SSerial.print(adata);
  SSerial.print(' ');

  for (byte i = 0; i < 7; i++) {
    // Serial.print(data[i]);
    // Serial.print(' ');
    SSerial.print(data[i]);
    SSerial.print(' ');
  }

  // Serial.println();
  SSerial.print('#');
  delay(50);
}

void getData() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  for (byte i = 0; i < 7; i++) {
    data[i] = Wire.read() << 8 | Wire.read();
  }
}