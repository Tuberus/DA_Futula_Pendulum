#pragma once

#include <cmath>
#include "freertos/FreeRTOS.h"
#include "esp_rom_sys.h"
#include "FloatFilt.h"
#include "MPU_Esp.h"

constexpr int x_idx = 0;
constexpr int y_idx = 1;
constexpr int z_idx = 2;
constexpr int NUM_AXES = 3;

namespace
{
  inline constexpr int CAL_SAMPLES = 500;
  inline constexpr int CAL_DELAY_US = 3000;

  inline constexpr float DT_SENS = 1e-3f;
  inline constexpr float ACC_MAX = 2050.0f;
  inline constexpr float GY_ANGLE_FACT = (2000.0f / 32767.0f) * 10.0f;
  inline constexpr float ALPHA = 0.001f;
  inline constexpr float BETA = 1.0f - ALPHA;
  inline constexpr float RAD_GRAD = 180.0f / static_cast<float>(M_PI);
}

class ImuChan
{
public:
  enum class Type
  {
    Acc = 1,
    Gyro = 2
  };

  ImuChan(int *aValAry, Type type)
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

  ~ImuChan()
  {
    for (int i = 0; i < NUM_AXES; i++)
    {
      delete f1[i];
      delete f2[i];
    }
  }

  ImuChan(const ImuChan &) = delete;
  ImuChan &operator=(const ImuChan &) = delete;

  void CalcFilt()
  {
    for (int i = 0; i < NUM_AXES; i++)
      CalcFilt(i);
  }

  void CalcFilt(int idx)
  {
    f1[idx]->CalcOneStep(ary[idx]);
    f2[idx]->CalcOneStep(ary[idx]);
  }

  int getVal(int idx) const
  {
    return static_cast<int>(ary[idx] - offset[idx]);
  }

  float getFilt1(int idx) const
  {
    return f1[idx]->y - offset[idx];
  }

  float getFilt2(int idx) const
  {
    return f2[idx]->yn - offset[idx];
  }

  void SetAlpha(float aAlpha)
  {
    for (int i = 0; i < NUM_AXES; i++)
      f1[i]->SetAlpha(aAlpha);
  }

  void CalGyro(bool readSensor = true)
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

private:
  Tp1Ord *f1[NUM_AXES];
  Tp2OrdF *f2[NUM_AXES];
  int *ary;
  float offset[NUM_AXES];
};

class KalmSimple
{
public:
  float accAngle;
  float omega;
  float complAngle;
  float gySum;

  KalmSimple() { Reset(); }

  void Reset()
  {
    accAngle = omega = complAngle = gySum = 0.0f;
  }

  float getAbsAngle() const { return std::fabs(complAngle); }

  void CalcFilter(float accVal, float gyroVal)
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
};
