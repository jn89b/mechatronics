// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050

// User parameters set whatever you want
const int AccRange = 0;
const int GyroRange = 0;


int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
float FX ;
float FY ;
float FZ ;

float AccConversion;
float GyroConversion;

float mapAccRange()
{
  switch(AccRange){
    case 0: return 16384;
    case 1: return 8192;
    case 2: return 4096;
    case 3: return 2048;
    default: return 16384;
  }
}

float mapGyroRange()
{
  switch(GyroRange){
    case 0: return 131;
    case 1: return 65.5;
    case 2: return 32.8;
    case 3: return 16.4;
    default: return 131;
  }
}

void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  // Set full-scale range for the gyroscope to ±250 dps
  setGyroRange(GyroRange);  // 0 = ±250 dps, 1 = ±500 dps, 2 = ±1000 dps, 3 = ±2000 dps
  GyroConversion = mapGyroRange();

  // Set accelerometer range to ±4g
  setAccRange(AccRange);  // Use 0 for ±2g, 1 for ±4g, 2 for ±8g, and 3 for ±16g
  AccConversion = mapAccRange();

  Serial.begin(9600);
}

void setGyroRange(int range) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B);    // GYRO_CONFIG register
  Wire.write(range << 3);  // Set FS_SEL bits (bits 3 and 4)
  Wire.endTransmission(true);
}

void setAccRange(int range) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C);    // ACC_CONFIG register
  Wire.write(range << 3);  // Set FS_SEL bits (bits 3 and 4)
  Wire.endTransmission(true);
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers

  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  FX = AcX/16384;
  FY = AcY/16384;
  FZ = AcZ/16384;
  Serial.print("AcX = "); Serial.print(AcX/AccConversion);
  Serial.print(" | AcY = "); Serial.print(AcY/AccConversion);
  Serial.print(" | AcZ = "); Serial.print(AcZ/AccConversion);
  // Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX/GyroConversion);
  Serial.print(" | GyY = "); Serial.print(GyY/GyroConversion);
  Serial.print(" | GyZ = "); Serial.println(GyZ/GyroConversion);
  delay(10);
}