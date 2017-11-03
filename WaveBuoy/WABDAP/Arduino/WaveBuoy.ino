#include <Wire.h>
#include "Timer.h"

#define GPS 8
#define LED 9

#define GYRO 0x68  //i2C address
#define REG_GYRO_X 0x1D   // IMU-3000 Register address for GYRO_XOUT_H
#define ACCEL 0x0F        // Accel I2c Address
#define ACCEL_FILT_REG 0x21   // Data control reg addr
#define ACCEL_HPFHZ_100 0x10 // HPF roll off at 100 Hz
#define ACCEL_LPFHZ_25 0x2  // LPF roll off at 25 hz
#define KXTF9_POWER_CTL 0x1B  // Power control reg addr
#define KXTF9_PC0 0x80        // Operating mode bit
#define KXTF9_GS2G 0x00       // bits for 2G mode
#define KXTF9_GS4G 0x08       // bits for 4G mode
#define KXTF9_GS8G 0x10       // bits for 8G mode
#define magnetometer 0x1E //0011110b, I2C 7bit address of HMC5883
#define MAGREGB  (byte)0x1
#define MAGGAIN0 (byte)0x00  // 0.9 Ga
#define MAGGAIN1 (byte)0x20  // 1.2 Ga (default)
#define MAGGAIN2 (byte)0x40  // 1.9 Ga
#define MAGGAIN3 (byte)0x60  // 2.5 Ga
#define MAGGAIN4 (byte)0x80  // 4.0 Ga
#define MAGGAIN5 (byte)0xA0  // 4.6 Ga
#define MAGGAIN6 (byte)0xC0  // 5.5 Ga
#define MAGGAIN7 (byte)0xE0  // 7.9 Ga
#define MAGMODE  0x2

// Serial
//String incomingData;
//boolean stringComplete;

// Wire
byte buffer[12];   // Array to store ADC values 
int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;
int mag_x, mag_y, mag_z;

Timer timer;
long milliseconds;
int i;

void setup()
{
  delay(200);
  
  pinMode(GPS, OUTPUT);       
  pinMode(LED, OUTPUT);       
  digitalWrite(GPS, LOW);
  digitalWrite(LED, LOW);
 // Serial
 // incomingData.reserve(200);
  //if (Serial)
    //Serial.println(F("Serial Initialized"));
  
  // Wire
  Wire.begin(); delay(10);
  writeTo(GYRO, 0x16, 0x0B); delay(10);
  writeTo(GYRO, 0x18, 0x06); delay(10);
  writeTo(GYRO, 0x14, ACCEL); delay(10);
  writeTo(GYRO, 0x3D, 0x08); delay(10);
  writeTo(ACCEL, KXTF9_POWER_CTL, KXTF9_PC0 | KXTF9_GS2G); delay(10);   
  writeTo(GYRO, 0x3D, 0x28); delay(10);
  writeTo(magnetometer, 0x02, 0x00);delay(10);
  //Serial.println(F("Wire Initialized"));
  
  // Timer
  timer.every(20, sample);
   delay(100);
  Serial.begin(57600);
}

void loop()
{
  //incomingData = "";
  //stringComplete = false;
  timer.update();
}

/*
void serialEvent()
{
  while (Serial.available()) {
    char inChar = (char)Serial.read(); 
    incomingData += inChar;
    if (inChar == '\n')
      stringComplete = true;
  }
}*/

void writeTo(int device, byte address, byte val) {
  Wire.beginTransmission(device); // start transmission to device 
  Wire.write(address);             // send register address
  Wire.write(val);                 // send value to write
  Wire.endTransmission();         // end transmission
}

void sample()
{
  long iTime = millis() - milliseconds;   
  // Read the Gyro X, Y and Z and Accel X, Y and Z all through the gyro
  // First set the register start address for X on Gyro  
      Wire.beginTransmission(GYRO);
      Wire.write(REG_GYRO_X); //Register Address GYRO_XOUT_H
      Wire.endTransmission();
  // New read the 12 data bytes
      Wire.beginTransmission(GYRO);
      Wire.requestFrom(GYRO,12); // Read 12 bytes
      i = 0;
      while(Wire.available())
      {
          buffer[i] = Wire.read();
          i++;
      }
      Wire.endTransmission();
      Wire.beginTransmission(magnetometer);
      Wire.write(0x03); //select register 3, X MSB register
      Wire.endTransmission();

      Wire.requestFrom(magnetometer, 6);
      if(6<=Wire.available()){
      mag_x = Wire.read() << 8; //X msb
      mag_x |= Wire.read(); //X lsb
      mag_z = Wire.read() << 8; //Z msb
      mag_z |= Wire.read(); //Z lsb
      mag_y = Wire.read() << 8; //Y msb
      mag_y |= Wire.read(); //Y lsb
    }

  //Combine bytes into integers
  // Gyro format is MSB first
      gyro_x = buffer[0] << 8 | buffer[1];
      gyro_y = buffer[2] << 8 | buffer[3];
      gyro_z = buffer[4] << 8 | buffer[5];
  // Accel is LSB first. Also because of orientation of chips
  // accel y output is in same orientation as gyro x
  // and accel x is gyro -y
      accel_y = buffer[7] << 8 | buffer[6];
      accel_x = buffer[9] << 8 | buffer[8];
      accel_z = buffer[11] << 8 | buffer[10];

      Serial.print(iTime);
      Serial.print(" ");
      Serial.print(gyro_x);  // echo the number received to screen
      Serial.print(" ");
      Serial.print(gyro_y);  // echo the number received to screen
      Serial.print(" ");
      Serial.print(gyro_z);  // echo the number received to screen 
      Serial.print(" ");
      Serial.print(accel_x);  // echo the number received to screen
      Serial.print(" ");
      Serial.print(accel_y);  // echo the number received to screen
      Serial.print(" ");
      Serial.print(accel_z);  // echo the number received to screen    
      Serial.print(" ");
      Serial.print(mag_x);  // echo the number received to screen
      Serial.print(" ");
      Serial.print(mag_y);  // echo the number received to screen
      Serial.print(" ");  
      Serial.print(mag_z);  // echo the number received to screen    
      Serial.println("");     // prints carriage return
}
