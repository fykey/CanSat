#ifndef BMX_READ_H_INCLUDE
#define BMX_READ_H_INCLUDE
#include <Wire.h>
// BMX055 加速度センサのI2Cアドレス  
#define Addr_Accl 0x19  // (JP1,JP2,JP3 = Openの時)
// BMX055 ジャイロセンサのI2Cアドレス
#define Addr_Gyro 0x69  // (JP1,JP2,JP3 = Openの時)
// BMX055 磁気センサのI2Cアドレス
#define Addr_Mag 0x13   // (JP1,JP2,JP3 = Openの時)



class BMX_Read {
    public:

        // センサーの値を保存する
        float xAccl = 0.00;
        float yAccl = 0.00;
        float zAccl = 0.00;
        float xGyro = 0.00;
        float yGyro = 0.00;
        float zGyro = 0.00;
        int   xMag  = 0;
        int   yMag  = 0;
        int   zMag  = 0;

        BMX_Read(); // コンストラクタ
        void BMX055_Accl();
        void BMX055_Gyro();
        void BMX055_Mag();


    private:
    

}



# endif