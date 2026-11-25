#pragma once

#include "MPU_Esp.h"
#include "FloatFilt.h"
// esp_rom_delay_us()
#include "freertos/FreeRTOS.h"
#include "math.h"

extern MPU6050 mpu;

const int x_idx = 0;
const int y_idx = 1;
const int z_idx = 2;

class ImuChan {
public:
  // type 1..acc, 2..gyro
  ImuChan(int* aValAry, int type);
  void CalcFilt();
  void CalcFilt(int idx);
  int getVal(int idx);

  // narrow BW 1Ord
  float getFilt1(int idx); // Tp1Ord
  // broad BW 2nd Ord
  float getFilt2(int idx); // Tp2Ord
  void SetAlpha(float aAlpha); // Tp1Ord

  void CalGyro(bool readSensor=true);
private:
  Tp1Ord* f1[3];
  Tp2OrdF* f2[3];
  int* ary;
  int offset[3];
};


class KalmSimple {
public:
  float accAngle;
  float omega;
  float complAngle; // kalman filtert
  float gySum;
public:
  KalmSimple() 
  {
    Reset();
  }
  void Reset()
  {
    accAngle = omega = complAngle = gySum = 0;
  }
  float getAbsAngle()
  {
    return fabs(complAngle);
  }
  void CalcFilter(float accVal, float gyroVal);
};


// ---------------------------------------------------------------------------
// Implementation (formerly ImuAlgo.cpp)
// ---------------------------------------------------------------------------

inline void ImuChan::CalGyro(bool readSensor)
{
  int sum[3], i;
  for (i = 0; i < 3; i++) {
    sum[i] = 0;
    this->offset[i] = 0;
  }
  
  for (int nCal = 1; nCal <= 500; nCal++) {
    esp_rom_delay_us(3000);
    if( readSensor )
      mpu.getGyro(); 
    this->CalcFilt();
    for (i = 0; i < 3; i++)
      sum[i] += this->getFilt2(i);
  }

  for (i = 0; i < 3; i++)
    this->offset[i] = sum[i] / 500;
}

inline ImuChan::ImuChan(int* aValAry, int type)
{
  ary = aValAry;
  int i;
  for (i = 0; i < 3; i++) {
    offset[i] = 0;
    f1[i] = new Tp1Ord();
    f2[i] = new Tp2OrdF();
  }
  for (i = 0; i < 3; i++) {
    if( type==1 )
      f2[i]->Init(TP_COE_p010); // acc
    else
      f2[i]->Init(TP_COE_p05); // gyro
  }
}

inline void ImuChan::SetAlpha(float aAlpha)
{
  for(int i=0; i<3; i++)
    f1[i]->SetAlpha(aAlpha);
}

inline void ImuChan::CalcFilt()
{
  for (int i = 0; i < 3; i++) {
    f1[i]->CalcOneStep(ary[i]);
    f2[i]->CalcOneStep(ary[i]);
  }
}

inline void ImuChan::CalcFilt(int idx)
{
  f1[idx]->CalcOneStep(ary[idx]);
  f2[idx]->CalcOneStep(ary[idx]);
}

inline int ImuChan::getVal(int idx)
{
  return ary[idx] - offset[idx];
}

inline float ImuChan::getFilt1(int idx)
{
  return f1[idx]->y - (float)offset[idx];
}

inline float ImuChan::getFilt2(int idx)
{
  return f2[idx]->yn - (float)offset[idx];
}


const float DT_SENS = 1E-3;
const float ACC_MAX = 2050; // -2000..2105
const float GY_ANGLE_FACT = (2000.0 / 32767.0) * 10.0;
const float ALPHA = 0.001f; // 0.02
const float BETA = 1.0f - ALPHA;
const float RAD_GRAD = 180.0f/(float)M_PI;

inline void KalmSimple::CalcFilter(float accVal, float gyroVal)
{
  if (accVal > ACC_MAX) accVal = ACC_MAX;
  if (accVal < -ACC_MAX) accVal = -ACC_MAX;
  accAngle = 10.0f * RAD_GRAD * asin((float)accVal / (float)ACC_MAX);
  omega = GY_ANGLE_FACT * gyroVal;
  
  gySum += omega * DT_SENS;
  complAngle = BETA*(complAngle + omega*DT_SENS) + ALPHA*accAngle;
}