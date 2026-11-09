#pragma once

#include "RTEnvHL.h"
#include "TimerHL.h"
#include <cmath>
#include "Features.h"
#include "driver/gpio.h"

class RcServo {
public:
  int offset;
  int pw;
public:
  static void InitTimer()
  {
    tim=InitPwmTimer(0, 1000000, 20000);
  }
  static void StartTimer()
  {
    ::StartPwmTimer(tim);
  }

  RcServo(int aPin)
  {
    _pin = aPin;
    offset=1300; pw=0;
  }
  void Init()
  {
    hpwm=InitPwm(tim, _pin);
  }
  void SetPw(uint32_t width)
  {
    pw = width;
    ::SetPw(hpwm,offset+width);
  }
  void SetPercent(float aVal)
  {
    SetPwPercent(hpwm, aVal);
  }
private:
  int _pin;
  PwmHL hpwm;
  inline static PwmTim tim;
};


class Motor {
public:
  float pow;
  float dir;
  bool inv;
public:
  static void InitTimer()
  {
    ChkFeature2(FEAT_SEM);
    tim = InitPwmTimer(0, 1000000, 1000);
  }

  static void StartTimer()
  {
    ChkFeature2(FEAT_SEM);
    ::StartPwmTimer(tim);
  }

  Motor(int aPwm, int aFwd, int aRev=-1)
  {
    _pwm = aPwm; _fwd = aFwd; _rev = aRev;
    pow = 0; dir = 0; inv = false;
  }

  void Init()
  {
    ChkFeature2(FEAT_SEM);
    hpwm = InitPwm(tim, _pwm);
    GpIoInit(1ULL << _fwd);
    if (_rev != -1)
      GpIoInit(1ULL << _rev);
  }

  void setPow(float aPow)
  {
    if (_rev == -1)
      return;
    if (aPow > 0) {
      gpio_set_level((gpio_num_t)_fwd, 1);
      gpio_set_level((gpio_num_t)_rev, 0);
      dir = true;
    }
    else {
      aPow = -aPow;
      gpio_set_level((gpio_num_t)_fwd, 0);
      gpio_set_level((gpio_num_t)_rev, 1);
      dir = false;
    }
    SetPwPercent(hpwm, aPow);
  }
  
  // Tumbler Motor
  void setPow2(float aPow)
  {
    if (aPow > 0.95) aPow = 0.95;
    if (aPow < -0.95) aPow = -0.95;
    pow = aPow;
    if (aPow >= 0) {
      setDirPin(false);
    }
    else {
      aPow = -aPow;
      setDirPin(true);
    }
    SetPwPercent(hpwm, aPow);
    if (pow > 0) dir = 1.0;
    else if (pow < 0) dir = -1.0;
    else dir = 0.0;
  }

  void setDirPin(bool dir)
  {
    bool lvl = (dir != inv);
    gpio_set_level((gpio_num_t)_fwd, lvl ? 1 : 0);
    if (_rev != -1)
      gpio_set_level((gpio_num_t)_rev, lvl ? 0 : 1);
  }

private:
  int _pwm, _fwd, _rev;
  PwmHL hpwm;
  inline static PwmTim tim;
};


class RateLim {
public:
  float F_SAMPLE;
public:
  float inTmp;
  float in, out;
  int16_t goesUp;
  int16_t on;
public:
  RateLim(float aRate)
  {
    F_SAMPLE = 300.0;
    in = out = 0; on = 1; SetRateSec(aRate);
  }
  void SaveIn()
  {
    inTmp = in;
  }
  void RestoreIn()
  {
    in = inTmp;
  }
  int IsFlat()
  {
    return out == in;
  }
  void Reset()
  {
    in = out = 0;
  }
  // 1000V in 300 Ticks = 1000/300
  void SetRateTicks(float aRate)
  {
    on = 1;
    if (aRate == 0) on = 0;
    _rate = aRate;
  }
  // eg. 1000     = 1000V/Sec
  // eg. 1000/10  = 1000V/10Sec
  // e.g 1000/0.1 = 1000V/0.1Sec
  void SetRateSec(float aRate)
  {
    on = 1;
    if (aRate == 0) on = 0;
    _rate = aRate / F_SAMPLE;
  }
  void CalcOneStep()
  {
    goesUp = 0;
    if (!on) {
      out = in; return;
    }
    if (fabs(out - in) <= _rate) {
      out = in; return;
    }
    if (in > 0 && in > out)
      goesUp = 1;
    if (in < 0 && in < out)
      goesUp = 1;
    if (in > out)
    {
      out += _rate;
    }
    else
    {
      out -= _rate;
    }
  }
public:
  float _rate;
};

// motor and encoder are one thing for the user
#include "Encoder2.h"
