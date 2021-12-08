#include "I2Cdev.h"                  // Подключение библиотеки I2Cdev
#include "MPU6050.h"                 // Подключение библиотеки MPU6050 
#include "Wire.h"                    // Подключение библиотеки WireCdev

#include <MAX3010x.h>

MAX30102 sensor;

#include <Adafruit_MLX90614.h>
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

MPU6050 CY531;                       // Создаем объект, символизирующий модуль датчика
int16_t ax, ay, az;                  // Переменные для хранения значений акселерометра
int16_t gx, gy, gz;                  // Переменные для хранения значений гироскоп

void setup()
{
  Wire.begin();                         // Инициализация Wire
  Serial.begin(115200);                  // Инициализация последовательного порта
//  Serial.println("Initializing I2C devices..."); // Печать текста
  CY531.initialize();                   // Инициализация MPU
  delay(100);                           // Пауза
//  Serial.println("Arduino MLX90614 Testing");  
  mlx.begin(); 
  
  if(sensor.begin()) { 
//    Serial.println("IR,Red");
  }
  else {
    Serial.println("MAX30102 Sensor not found");  
    while(1);
  }  
  delay(100);                           // Пауза
//  Serial.println("ax,ay,az,gx,gy,gz,TAC,TOC,TAF,TOF,pulse,tenzo,galv,ir,red");
  Serial.println("value");
}

#define bufl 6
char buff[bufl] = "-12345";


void i16t2buff (int16_t v){
  uint8_t i = 0;
  for (i = 0; i < bufl; i++){
    buff[i] = ' ';
  }
  
  if (v < 0){ 
    buff[0] = '-';
    v = -v;
  }

  i = bufl - 1;
  while (v != 0){
    buff[i] = 48 + (v % 10);
    v = v / 10;
    i--;
  }

  buff[i] = buff[0];
  buff[0] = ' ';
}

#define fltl 10
int16_t axf[fltl];
int16_t ayf[fltl];
int16_t azf[fltl];
int16_t gxf[fltl];
int16_t gyf[fltl];
int16_t gzf[fltl];

int16_t flt(int16_t v, int16_t * arr){
  uint8_t i;
  int32_t s = 0;
  for (i = fltl-1; i > 0; i--){ // shift >> 1
    arr[i] = arr[i-1];
    s += arr[i-1];
  }

  arr[0] = v;
  s += v;

  return s / fltl;
}

char toshow = 'p';

void toSerial(int16_t ax, int16_t ay, int16_t az, int16_t gx, int16_t gy, int16_t gz,
              float tambc, float tobjc, float tambf, float tobjf,
              uint16_t puls, uint16_t tenzo, uint16_t galvanic,
              uint32_t ir, uint32_t red)
{
  /*
  Serial.print(ax); Serial.print(",");
  Serial.print(ay); Serial.print(",");
  Serial.print(az); Serial.print(",");
  Serial.print(gx); Serial.print(",");
  Serial.print(gy); Serial.print(",");
  Serial.print(gz); Serial.print(",");
  Serial.print(tambc); Serial.print(",");
  Serial.print(tobjc); Serial.print(",");
  Serial.print(tambf); Serial.print(",");
  Serial.print(tobjf); Serial.print(",");
  Serial.print(puls); Serial.print(",");
  Serial.print(tenzo); Serial.print(",");
  Serial.print(galvanic); Serial.print(",");
  Serial.print(ir); Serial.print(",");
  Serial.print(red); 
  */
  switch (toshow) {
    case 'x' : Serial.print(ax); break;
    case 'y' : Serial.print(ay); break;
    case 'z' : Serial.print(az); break;
    case 'X' : Serial.print(gx); break;
    case 'Y' : Serial.print(gy); break;
    case 'Z' : Serial.print(gz); break;
    case 'T' : Serial.print(tobjc); break;
    case 'p' : Serial.print(puls); break;
    case 't' : Serial.print(tenzo); break;
    case 'g' : Serial.print(galvanic); break;
    case 'i' : Serial.print(ir); break;
    case 'r' : Serial.print(red); break;
    default: Serial.print(puls); break;
  }
  Serial.println();
}
              
void loop()
{
  CY531.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); // Чтение значений гироскопа и акселерометра
  auto sample = sensor.readSample(100);

  if (Serial.available() > 0) {
    toshow = Serial.read();
  }

// вывод значений в монитор
  toSerial(ax, ay, az, gx, gy, gz,
           mlx.readAmbientTempC(), mlx.readObjectTempC(), mlx.readAmbientTempF(), mlx.readObjectTempF(),
           analogRead(A0), analogRead(A1), analogRead(A2),
           sample.ir, sample.red
          );
/*
Serial.print("a/g:  ");

Serial.print("ax:"); i16t2buff(flt(ax, axf)); Serial.print(buff); Serial.print(" ");
Serial.print("ay:"); i16t2buff(flt(ay, ayf)); Serial.print(buff); Serial.print(" ");
Serial.print("az:"); i16t2buff(flt(az, azf)); Serial.print(buff); Serial.print(" ");
Serial.print("gx:"); i16t2buff(flt(gx, gxf)); Serial.print(buff); Serial.print(" ");
Serial.print("gy:"); i16t2buff(flt(gy, gyf)); Serial.print(buff); Serial.print(" ");
Serial.print("gz:"); i16t2buff(flt(gz, gzf)); Serial.print(buff); Serial.print(" ");
*/
/*
Serial.print("ax:"); i16t2buff(ax); Serial.print(buff); Serial.print(" ");
Serial.print("ay:"); i16t2buff(ay); Serial.print(buff); Serial.print(" ");
Serial.print("az:"); i16t2buff(az); Serial.print(buff); Serial.print(" ");
Serial.print("gx:"); i16t2buff(gx); Serial.print(buff); Serial.print(" ");
Serial.print("gy:"); i16t2buff(gy); Serial.print(buff); Serial.print(" ");
Serial.print("gz:"); i16t2buff(gz); Serial.print(buff); Serial.print(" ");
*/
/*
  Serial.print(" | *C Amb: "); Serial.print(mlx.readAmbientTempC()); Serial.print("  Obj: "); Serial.print(mlx.readObjectTempC());
  Serial.print(" | *F Amb: "); Serial.print(mlx.readAmbientTempF()); Serial.print("  Obj: "); Serial.print(mlx.readObjectTempF());

  Serial.print(" | Pls: "); i16t2buff(analogRead(A0)); Serial.print(buff);
  Serial.print(" | Tnz: "); i16t2buff(analogRead(A1)); Serial.print(buff);
  Serial.print(" | Glv: "); i16t2buff(analogRead(A2)); Serial.print(buff);

  auto sample = sensor.readSample(100);
  Serial.print(" | IR,RED: ");
  Serial.print(sample.ir);
  Serial.print(",");
  Serial.print(sample.red);

Serial.println();
*/
//delay(50); 
}
