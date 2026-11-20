#pragma once

#include "RTEnvHL.h"
#include "TimerHL.h"
#include "FloatFilt.h"
#include "Features.h"
#include <cmath>

inline int AbsDiff(uint32_t a, uint32_t b)
{
  int diff = a - b;
  return (diff < 0) ? -diff : diff;
}

inline bool IsFrequLow(uint32_t aPw)
{
  return aPw > 5000;
}

inline bool Is_F_gt_500(uint32_t aPw)
{
  return aPw < 2000;
}

inline int frequDiff(uint32_t pw1, uint32_t pw2)
{
  const uint32_t FREQU_FACT_I = 1E6;
  int diff = (int)(FREQU_FACT_I / pw1) - (int)(FREQU_FACT_I / pw2);
  return (diff < 0) ? -diff : diff;
}

class Encoder2
{
  const float FREQU_FACT = 1E6;
  const float DT_FACT = 150.0;

  static void IRAM_ATTR encoder_isr(void* arg)
  {
    ((Encoder2*)arg)->ISRFunction();
  }

public:
  Encoder2(int aInt, int aDir)
  {
    _intrPin = aInt;
    _dirPin = aDir;
    inv = false;
    actDir = 1;
    cnt = 0;
    _t1 = 0;
    pw = 0x0FFFFFF;
    _runFlag = 0;
    filtOn = true;
    igCnt = 0;
    z1 = z2 = z3 = diff = 0;
    info = 0;
    _filt = new Tp2OrdF();
    _filt->Init(TP_COE_p010);
  }

  void Init()
  {
    ChkFeature2(FEAT_SEM);
    GpIoInit(1ULL << _dirPin, true);
    GpIoInitInterrupt(_intrPin, encoder_isr, this, GPIO_INTR_POSEDGE);
  }

  void ISRFunction()
  {
    checkDir();
    _runFlag = 1;
    int64_t t2 = esp_timer_get_time();
    int64_t actPw = t2 - _t1;
    _t1 = t2;

    info = 0;
    if (IsFrequLow(pw)) {
      info = 200;
      if (Is_F_gt_500(actPw))
        return;
    }

    if (actPw < 600)
      return;
    pw = actPw;
  }

  void setBW(int aFiltNum)
  {
    filtOn = true;
    if (aFiltNum < 1 || aFiltNum > 5) {
      filtOn = false;
      return;
    }
    switch (aFiltNum) {
    case 1:
      _filt->Init(TP_COE_p010); break;
    case 2:
      _filt->Init(TP_COE_p025); break;
    case 3:
      _filt->Init(TP_COE_p04); break;
    case 4:
      _filt->Init(TP_COE_p05); break;
    case 5:
      _filt->Init(TP_COE_p100); break;
    default:
      _filt->Init(TP_COE_p010);
    }
  }

  void CalcFilt2()
  {
    int64_t actPw = esp_timer_get_time() - _t1;
    if (actPw > 50000 && _runFlag != 0) {
      _runFlag = 0;
      pw = 0x0FFFFFF;
    }
    if (actDir > 0)
      _filt->CalcOneStep((1.0 / (float)pw) * FREQU_FACT);
    else
      _filt->CalcOneStep((-1.0 / (float)pw) * FREQU_FACT);
  }

  void CalcDiff(float aVal)
  {
    diff = aVal - z3;
    z3 = z2;
    z2 = z1;
    z1 = aVal;
  }

  float getFrequUS()
  {
    return (1.0 / (float)pw) * FREQU_FACT;
  }

  float getFrequ()
  {
    if (actDir > 0)
      return (1.0 / (float)pw) * FREQU_FACT;
    else if (actDir < 0)
      return (-1.0 / (float)pw) * FREQU_FACT;
    else
      return 0.0;
  }

  float getFrequF()
  {
    if (!filtOn)
      return getFrequ();
    return _filt->yn;
  }

  float getDiff()
  {
    return diff * DT_FACT;
  }

  int getPW()
  {
    return (pw > 32000) ? 32000 : pw;
  }

  int getDir()
  {
    return actDir;
  }

  void checkDir()
  {
    if (!inv) {
      if (gpio_get_level((gpio_num_t)_dirPin)) {
        cnt++;
        actDir = 1;
      } else {
        cnt--;
        actDir = -1;
      }
    } else {
      if (gpio_get_level((gpio_num_t)_dirPin)) {
        cnt--;
        actDir = -1;
      } else {
        cnt++;
        actDir = 1;
      }
    }
  }

  int absCnt()
  {
    return ((int)cnt < 0) ? -(int)cnt : cnt;
  }

private:
  int _dirPin, _intrPin;
  float actDir;
  bool filtOn;
  int64_t _t1;
  uint32_t _runFlag;
  int igCnt;
  Tp2OrdF *_filt;
  float z1, z2, z3;

public:
  int cnt;
  uint32_t pw;
  float diff;
  bool inv;
  int info;
};