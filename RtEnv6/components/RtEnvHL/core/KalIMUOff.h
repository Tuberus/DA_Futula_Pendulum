#pragma once

#include <cmath>

constexpr int x_idx = 0;
constexpr int y_idx = 1;
constexpr int z_idx = 2;
constexpr int NUM_AXES = 3;

class Tp1Ord;
class Tp2OrdF;

class ImuChan
{
public:
  enum class Type
  {
    Acc = 1,
    Gyro = 2
  };

  ImuChan(int *aValAry, Type type);
  ~ImuChan();

  ImuChan(const ImuChan &) = delete;
  ImuChan &operator=(const ImuChan &) = delete;

  void CalcFilt();
  void CalcFilt(int idx);

  int getVal(int idx) const;
  float getFilt1(int idx) const;
  float getFilt2(int idx) const;
  void SetAlpha(float aAlpha);

  void CalGyro(bool readSensor = true);

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

  void CalcFilter(float accVal, float gyroVal);
};