#include "Wire.h"                    // Подключение библиотеки WireCdev
#include <MAX3010x.h>


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
  adc_1_galvanic_begin();
  giroskop_axelerometer_begin();
}

void loop() {
  pulsometer_read(100);
  adc_1_galvanic_read();
  giroskop_axelerometer_read();
  tenzo_line_read();

  Serial.print(adc_1_galvanic_value); Serial.print(" ");
  Serial.print(adc_1_galvanic_volt); Serial.print(" ");

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
  // delay(100);
}

// ########################################################
const int MPU_addr = 0x68; // адрес датчика

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
#define adc_1_galvanic__ADDRESS	0x68

void adc_1_galvanic_begin () {
  Wire.beginTransmission(adc_1_galvanic__ADDRESS);//i2c адрес MCP3421= B1101000
  Wire.write(B11000); // настройка АЦП: постоянное преобразование (1), 16бит(10), усиление=1(00). https://static.chipdip.ru/lib/011/DOC013011388.pdf str 11
  Wire.endTransmission();

  // https://arduino.ru/forum/apparatnye-voprosy/mini-obzor-atsp-mcp3421-s-vykhodom-i2c 
}

void adc_1_galvanic_read () {
  Wire.requestFrom(adc_1_galvanic__ADDRESS,3); //запросить 3 байта данных
  adc_1_galvanic_value = 0;
  adc_1_galvanic_value = Wire.read();//упаковка в одну переменную.
  adc_1_galvanic_value= ((adc_1_galvanic_value<<8)| Wire.read()); //упаковка в одну переменную.
  adc_1_galvanic_value= ((adc_1_galvanic_value<<8)| Wire.read()); //упаковка в одну переменную.
  adc_1_galvanic_volt = adc_1_galvanic_value * 2.048 /131072 ; // LSB=15uV
}