#include "I2Cdev.h"                  // Подключение библиотеки I2Cdev
#include "MPU6050.h"                 // Подключение библиотеки MPU6050 
#include "Wire.h"                    // Подключение библиотеки WireCdev
#include <MAX3010x.h>


MAX30102 sensor;


void setup() {
  Wire.begin();                         // Инициализация Wire
  Serial.begin(115200);                 // Инициализация последовательного порта
  max30102_begin();
}

void loop() {
  auto sample = sensor.readSample(100);
  Serial.print(sample.ir,10); Serial.print(" "); Serial.print(sample.red); Serial.println(""); 
}


void max30102_begin ()
{
  if( ! sensor.begin()) { 
    Serial.println("MAX30102 Sensor not found");  
    while(1);
  }  
}