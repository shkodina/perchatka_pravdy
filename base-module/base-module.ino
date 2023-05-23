#include "I2Cdev.h"                  // Подключение библиотеки I2Cdev
#include "MPU6050.h"                 // Подключение библиотеки MPU6050 
#include "Wire.h"                    // Подключение библиотеки WireCdev
#include <MAX3010x.h>


MAX30102 pulsometer;
long pulsometer_red, pulsometer_ir;

MPU6050 giroskop_axelerometer;                   
int16_t ax, ay, az;                  // Переменные для хранения значений акселерометра
int16_t gx, gy, gz;                  // Переменные для хранения значений гироскоп

int16_t tenzo_line;



void setup() {
  Wire.begin();                         // Инициализация Wire
  Serial.begin(115200);                 // Инициализация последовательного порта

  giroskop_axelerometer.initialize();                // Инициализация MPU
  pulsometer_begin();
}

void loop() {
  pulsometer_read(100);
  giroskop_axelerometer.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
  tenzo_line_read();
  
  Serial.print(tenzo_line); Serial.print(" ");

  Serial.print(pulsometer_ir); Serial.print(" ");
  Serial.print(pulsometer_red); Serial.print(" ");

  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); Serial.print(" ");
  
  Serial.println("");

}

// ########################################################

void pulsometer_begin ()
{
  if( ! pulsometer.begin()) { 
    Serial.println("pulsometer MAX30102 Sensor not found");  
    while(1);
  }  
}

void pulsometer_read(int v)
{
  auto sample = pulsometer.readSample(v);
  pulsometer_red = sample.red;
  pulsometer_ir = sample.ir;
}

// #######################################################

int16_t tenzo_line_read ()
{
  tenzo_line = analogRead(A0);
  return tenzo_line;
}