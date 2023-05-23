#include "Wire.h"                    // Подключение библиотеки WireCdev
#include <MAX3010x.h>

#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 termo_face = Adafruit_MLX90614();

MAX30102 pulsometer;
long pulsometer_red, pulsometer_ir;

int16_t ax, ay, az, agt, gx, gy, gz;                  // Переменные для хранения значений акселерометра гироскопа

int16_t tenzo_line;

int32_t adc_1_galvanic_value, adc_2_galvanic_value;
float   adc_1_galvanic_volt,  adc_2_galvanic_volt;

void setup() {
  Wire.begin();                         // Инициализация Wire
  Serial.begin(115200);                 // Инициализация последовательного порта

  pulsometer_begin();
  delay(100);
  adc_1_galvanic_begin();
  adc_2_galvanic_begin();
  giroskop_axelerometer_begin();
  delay(100);
  termo_face.begin(); 

}

void loop() {
  pulsometer_read(100);
  adc_1_galvanic_read();
  adc_2_galvanic_read();
  giroskop_axelerometer_read();
  tenzo_line_read();

  Serial.print(adc_1_galvanic_value); Serial.print(" ");
  Serial.print(adc_1_galvanic_volt); Serial.print(" ");

  Serial.print(adc_2_galvanic_value); Serial.print(" ");
  Serial.print(adc_2_galvanic_volt); Serial.print(" ");

  Serial.print(tenzo_line); Serial.print(" ");

  Serial.print(pulsometer_ir); Serial.print(" ");
  Serial.print(pulsometer_red); Serial.print(" ");

  Serial.print(ax); Serial.print(" ");
  Serial.print(ay); Serial.print(" ");
  Serial.print(az); Serial.print(" ");
  Serial.print(gx); Serial.print(" ");
  Serial.print(gy); Serial.print(" ");
  Serial.print(gz); Serial.print(" ");

  Serial.print(termo_face.readAmbientTempC()); Serial.print(" ");
  Serial.print(termo_face.readObjectTempC() ); Serial.print(" ");
  Serial.print(termo_face.readAmbientTempF()); Serial.print(" ");
  Serial.print(termo_face.readObjectTempF() ); Serial.print(" ");

  
  Serial.println("");
  // delay(100);
}

// ########################################################
// https://alexgyver.ru/arduino-mpu6050/
#define MPU_addr 0x68 // адрес датчика

void giroskop_axelerometer_begin ()
{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void giroskop_axelerometer_read () {
  // массив данных
  // [accX, accY, accZ, temp, gyrX, gyrY, gyrZ]
  // acc - ускорение, gyr - угловая скорость, temp - температура (raw)
  int16_t data[7]; 
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  for (byte i = 0; i < 7; i++) {
    data[i] = Wire.read() << 8 | Wire.read();
  }
  ax =  data[0];
  ay =  data[1];
  az =  data[2];
  agt = data[3];
  gx =  data[4];
  gy =  data[5];
  gz =  data[6];
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

// #######################################################
// https://arduino.ru/forum/apparatnye-voprosy/mini-obzor-atsp-mcp3421-s-vykhodom-i2c 

#define adc_1_galvanic__ADDRESS	0x68

void adc_1_galvanic_begin () {
  Wire.beginTransmission(adc_1_galvanic__ADDRESS);//i2c адрес MCP3421= B1101000
  Wire.write(B11000); // настройка АЦП: постоянное преобразование (1), 16бит(10), усиление=1(00). https://static.chipdip.ru/lib/011/DOC013011388.pdf str 11
  Wire.endTransmission();
}

void adc_1_galvanic_read () {
  Wire.requestFrom(adc_1_galvanic__ADDRESS,3); //запросить 3 байта данных
  adc_1_galvanic_value = 0;
  adc_1_galvanic_value = Wire.read();//упаковка в одну переменную.
  adc_1_galvanic_value= ((adc_1_galvanic_value<<8)| Wire.read()); //упаковка в одну переменную.
  adc_1_galvanic_value= ((adc_1_galvanic_value<<8)| Wire.read()); //упаковка в одну переменную.
  adc_1_galvanic_volt = adc_1_galvanic_value * 2.048 /131072 ; // LSB=15uV
}

// #######################################################
// https://www.instructables.com/Arduino-and-the-TI-ADS1110-16-bit-ADC/

#define adc_2_galvanic__ADDRESS	0x48

void adc_2_galvanic_begin () {
  return;
}

void adc_2_galvanic_read () {
  int8_t highbyte, lowbyte, configRegister;

  Wire.requestFrom(adc_2_galvanic__ADDRESS, 3);
  while (Wire.available() ) // ensure all the data comes in
  {
    highbyte = Wire.read(); // high byte * B11111111
    lowbyte = Wire.read(); // low byte
    configRegister = Wire.read();
  }

  adc_2_galvanic_value = highbyte * 256;
  adc_2_galvanic_value = adc_2_galvanic_value + lowbyte;
  adc_2_galvanic_volt = adc_2_galvanic_value * 2.048 ;
  adc_2_galvanic_volt = adc_2_galvanic_volt / 32768.0;
}
