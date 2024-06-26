#include <./Arduino.h>
#include "BMX_READ.h"
#include <Wire.h>

void BMX_Read::BMX_Read(){

  Wire.begin();
     //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x0F); // Select PMU_Range register
  Wire.write(0x03);   // Range = +/- 2g
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x10);  // Select PMU_BW register
  Wire.write(0x08);  // Bandwidth = 7.81 Hz
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Accl);
  Wire.write(0x11);  // Select PMU_LPW register
  Wire.write(0x00);  // Normal mode, Sleep duration = 0.5ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x0F);  // Select Range register
  Wire.write(0x04);  // Full scale = +/- 125 degree/s
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x10);  // Select Bandwidth register
  Wire.write(0x07);  // ODR = 100 Hz
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Gyro);
  Wire.write(0x11);  // Select LPM1 register
  Wire.write(0x00);  // Normal mode, Sleep duration = 2ms
  Wire.endTransmission();
  delay(100);
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x83);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4B);  // Select Mag register
  Wire.write(0x01);  // Soft reset
  Wire.endTransmission();
  delay(100);
  //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4C);  // Select Mag register
  Wire.write(0x00);  // Normal Mode, ODR = 10 Hz
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x4E);  // Select Mag register
  Wire.write(0x84);  // X, Y, Z-Axis enabled
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x51);  // Select Mag register
  Wire.write(0x04);  // No. of Repetitions for X-Y Axis = 9
  Wire.endTransmission();
 //------------------------------------------------------------//
  Wire.beginTransmission(Addr_Mag);
  Wire.write(0x52);  // Select Mag register
  Wire.write(0x16);  // No. of Repetitions for Z-Axis = 15
  Wire.endTransmission();
}



void BMX_Read::BMX055_Accl()
{
    unsigned int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_Accl);
        Wire.write((2 + i));// Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Accl, 1);// Request 1 byte of data
        // Read 6 bytes of data
        // xAccl lsb, xAccl msb, yAccl lsb, yAccl msb, zAccl lsb, zAccl msb
        if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data to 12-bits
    xAccl = ((data[1] * 256) + (data[0] & 0xF0)) / 16;
    if (xAccl > 2047)  xAccl -= 4096;
    yAccl = ((data[3] * 256) + (data[2] & 0xF0)) / 16;
    if (yAccl > 2047)  yAccl -= 4096;
    zAccl = ((data[5] * 256) + (data[4] & 0xF0)) / 16;
    if (zAccl > 2047)  zAccl -= 4096;
    xAccl = xAccl * 0.0098; // range = +/-2g
    yAccl = yAccl * 0.0098; // range = +/-2g
    zAccl = zAccl * 0.0098; // range = +/-2g

    this->xAccl = xAccl;
    this->yAccl = yAccl;
    this->zAccl = zAccl;
}

void BMX_Read::BMX055_Gyro(){
    unsigned int data[6];
    for (int i = 0; i < 6; i++)
    {
        Wire.beginTransmission(Addr_Gyro);
        Wire.write((2 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Gyro, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        // xGyro lsb, xGyro msb, yGyro lsb, yGyro msb, zGyro lsb, zGyro msb
        if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data
    xGyro = (data[1] * 256) + data[0];
    if (xGyro > 32767)  xGyro -= 65536;
    yGyro = (data[3] * 256) + data[2];
    if (yGyro > 32767)  yGyro -= 65536;
    zGyro = (data[5] * 256) + data[4];
    if (zGyro > 32767)  zGyro -= 65536;

    xGyro = xGyro * 0.0038; //  Full scale = +/- 125 degree/s
    yGyro = yGyro * 0.0038; //  Full scale = +/- 125 degree/s
    zGyro = zGyro * 0.0038; //  Full scale = +/- 125 degree/s

}


void BMX_Read::BMX055_Mag(){
    unsigned int data[8];
    for (int i = 0; i < 8; i++)
    {
        Wire.beginTransmission(Addr_Mag);
        Wire.write((0x42 + i));    // Select data register
        Wire.endTransmission();
        Wire.requestFrom(Addr_Mag, 1);    // Request 1 byte of data
        // Read 6 bytes of data
        // xMag lsb, xMag msb, yMag lsb, yMag msb, zMag lsb, zMag msb
        if (Wire.available() == 1)
        data[i] = Wire.read();
    }
    // Convert the data
    xMag = ((data[1] <<5) | (data[0]>>3));
    if (xMag > 4095)  xMag -= 8192;
    yMag = ((data[3] <<5) | (data[2]>>3));
    if (yMag > 4095)  yMag -= 8192;
    zMag = ((data[5] <<7) | (data[4]>>1));
    if (zMag > 16383)  zMag -= 32768;
}
