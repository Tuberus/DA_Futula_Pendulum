#pragma once

#include "RTEnvHL.h"
#include "TimerHL.h"
#include "Features.h"
#include "Encoder2.h"

const int AI1 = 26, AI2 = 27;
const int ENC_A = 32, ENC_B = 33;

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

  void Init(){
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

inline MotorDRV8833 mot(AI1, AI2);
inline Encoder2 enc(ENC_A, ENC_B);

inline void InitMotors()
{
  MotorDRV8833::InitTimer();
  mot.Init();
  MotorDRV8833::StartTimer();
  mot.setPow(0.0f);

  enc.Init();

  mot.inv = false;
  enc.inv = false;
}