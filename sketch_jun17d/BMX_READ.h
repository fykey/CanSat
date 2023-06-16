///加速度系
float get_acclx();
float get_accly();
float get_acclz();
float get_accl_abs();

///地磁気系
float get_magx();
float get_magy();
float get_magz();

void BMX055_Init();

extern float xAccl;
extern float yAccl;
extern float zAccl;
extern float acc_abs;
