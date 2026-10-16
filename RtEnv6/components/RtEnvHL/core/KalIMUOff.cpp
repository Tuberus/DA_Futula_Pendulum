#include "KaliMUOff.h"
#include "MPU_Esp.h"
#include "FloatFilt.h"
#include "freertos/FreeRTOS.h"
#include <cmath>

extern MPU6050 mpu;

namespace
{
  constexpr int CAL_SAMPLES = 500;
  constexpr int CAL_DELAY_US = 3000;

  constexpr float DT_SENS = 1e-3f;
  constexpr float ACC_MAX = 2050.0f;
  constexpr float GY_ANGLE_FACT = (2000.0f / 32767.0f) * 10.0f;
  constexpr float ALPHA = 0.001f;
  constexpr float BETA = 1.0f - ALPHA;
  constexpr float RAD_GRAD = 180.0f / static_cast<float>(M_PI);
}

ImuChan::ImuChan(int *aValAry, Type type)
    : ary(aValAry)
{
  for (int i = 0; i < NUM_AXES; i++)
  {
    offset[i] = 0.0f;
    f1[i] = new Tp1Ord();
    f2[i] = new Tp2OrdF();
    if (type == Type::Acc)
      f2[i]->Init(TP_COE_p010);
    else
      f2[i]->Init(TP_COE_p05);
  }
}

ImuChan::~ImuChan()
{
  for (int i = 0; i < NUM_AXES; i++)
  {
    delete f1[i];
    delete f2[i];
  }
}

void ImuChan::CalGyro(bool readSensor)
{
  float sum[NUM_AXES] = {0.0f, 0.0f, 0.0f};

  for (int i = 0; i < NUM_AXES; i++)
    offset[i] = 0.0f;

  for (int n = 0; n < CAL_SAMPLES; n++)
  {
    esp_rom_delay_us(CAL_DELAY_US);
    if (readSensor)
      mpu.getGyro();
    CalcFilt();
    for (int i = 0; i < NUM_AXES; i++)
      sum[i] += getFilt2(i);
  }

  for (int i = 0; i < NUM_AXES; i++)
    offset[i] = sum[i] / static_cast<float>(CAL_SAMPLES);
}

void ImuChan::SetAlpha(float aAlpha)
{
  for (int i = 0; i < NUM_AXES; i++)
    f1[i]->SetAlpha(aAlpha);
}

void ImuChan::CalcFilt()
{
  for (int i = 0; i < NUM_AXES; i++)
    CalcFilt(i);
}

void ImuChan::CalcFilt(int idx)
{
  f1[idx]->CalcOneStep(ary[idx]);
  f2[idx]->CalcOneStep(ary[idx]);
}

int ImuChan::getVal(int idx) const
{
  return static_cast<int>(ary[idx] - offset[idx]);
}

float ImuChan::getFilt1(int idx) const
{
  return f1[idx]->y - offset[idx];
}

float ImuChan::getFilt2(int idx) const
{
  return f2[idx]->yn - offset[idx];
}

void KalmSimple::CalcFilter(float accVal, float gyroVal)
{
  if (accVal > ACC_MAX)
    accVal = ACC_MAX;
  if (accVal < -ACC_MAX)
    accVal = -ACC_MAX;

  accAngle = 10.0f * RAD_GRAD * asinf(accVal / ACC_MAX);
  omega = GY_ANGLE_FACT * gyroVal;

  gySum += omega * DT_SENS;
  complAngle = BETA * (complAngle + omega * DT_SENS) + ALPHA * accAngle;
}