#pragma once
#include "ControlBasics.h"
#include "TimerHL.h"
#include "Features.h"

class MotorDRV8833 {
public:
  float pow = 0.0f;
  bool inv = false;

  MotorDRV8833(int aIn1, int aIn2) : _in1(aIn1), _in2(aIn2) {}

  static void InitTimer() {
    ChkFeature2(FEAT_SEM);
    tim = InitPwmTimer(0, 1000000, 1000);
  }

  static void StartTimer() {
    ChkFeature2(FEAT_SEM);
    ::StartPwmTimer(tim);
  }

  void Init() {
    ChkFeature2(FEAT_SEM);
    hpwm1 = InitPwm(tim, _in1);
    hpwm2 = InitPwm(tim, _in2);
    SetPwPercent(hpwm1, 0.0f);
    SetPwPercent(hpwm2, 0.0f);
  }

  void setPow(float aPow) {
    setPow2(aPow);
  }

  void setPow2(float aPow) {
    if (aPow > 0.95f) aPow = 0.95f;
    if (aPow < -0.95f) aPow = -0.95f;
    pow = aPow;

    float cmd = inv ? -aPow : aPow;

    if (cmd > 0.001f) {
      SetPwPercent(hpwm1, cmd);
      SetPwPercent(hpwm2, 0.0f);
    } else if (cmd < -0.001f) {
      SetPwPercent(hpwm1, 0.0f);
      SetPwPercent(hpwm2, -cmd);
    } else {
      SetPwPercent(hpwm1, 0.0f);
      SetPwPercent(hpwm2, 0.0f);
    }
  }

private:
  int _in1, _in2;
  PwmHL hpwm1, hpwm2;
  inline static PwmTim tim;
};

class FurutaController : public PIDParam {

public:
  FurutaController(float aKP, float aKD, float aKI, float aKS, const char* aName)
    : PIDParam(aKP, aKD, aKI, aKS, aName)
  {
    pow = 0; on = 0; ang_1 = 0; diff = 0;
    angleOffs = 0; demPos = 0;
  }

  void CalcOneStep(float aAngle, float aSpeed, int actPos)
  {
    const float F_SAMP = 1000.0f;
    aAngle += angleOffs;
    diff = (aAngle - ang_1) * F_SAMP;
    ang_1 = aAngle;

    float posVal = 0.0f;
    if (actPos > 20 || actPos < -20)
      posVal = ClipHL(KI * (actPos - demPos), 0.7f);

    pow = KP * aAngle + KD * diff + KS * aSpeed + posVal;
    pow = ClipHL(pow, 0.95f);
  }
public:
  int demPos;
  float angleOffs;
public:
  float diff;
  float ang_1;
  float pow;
};

extern MPU6050 mpu;
ImuChan imuAcc(mpu.acc, ImuChan::Type::Acc);
ImuChan imuGyro(mpu.gyro, ImuChan::Type::Gyro);
KalmSimple kalm;
int dispMode = 1;
bool calFlag = false;

MotorDRV8833 motL(26, 27);
Encoder2 encL(32, 33);

inline void InitMotors()
{
  MotorDRV8833::InitTimer();
  motL.Init();
  MotorDRV8833::StartTimer();
  motL.setPow(0.0f);
  encL.Init();
  printf("Motors finished\n");
}