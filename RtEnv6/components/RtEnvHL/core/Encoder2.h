#pragma once

#include "RTEnvHL.h"
#include "TimerHL.h"
#include <cmath>
#include "FloatFilt.h"
#include "Features.h"
#include "esp_timer.h"
#include "driver/gpio.h"

inline int AbsDiff(uint32_t a, uint32_t b)
{
  int diff = a - b;
  if (diff < 0)
    return -diff;
  else
    return diff;
}

inline bool IsFrequLow(uint32_t aPw)
{
  // 50000 20Hz
  // 25000) 40Hz
  if (aPw > 5000) // 200Hz
    return true;
  return false;
}

inline bool Is_F_gt_500(uint32_t aPw)
{
  if (aPw < 2000) // 500Hz
    return true;
  return false;
}

inline int frequDiff(uint32_t pw1, uint32_t pw2)
{
  const uint32_t FREQU_FACT_I = 1E6;
  int diff = (int)(FREQU_FACT_I / pw1) - (int)(FREQU_FACT_I / pw2);
  if (diff < 0)
    return -diff;
  return diff;
}

class Encoder2 {
  const float FREQU_FACT = 1E6;
  const float DT_FACT = 150.0;
public:
  static void IRAM_ATTR encoder_isr(void* arg)
  {
    ((Encoder2*)arg)->ISRFunction();
  }

  Encoder2(int aInt, int aDir)
  {
    _intrPin = aInt; _dirPin = aDir; inv = false;
    actDir = 1;
    cnt = 0; _t1 = 0; pw = 0x0FFFFFF;
    _runFlag = 0; filtOn = true; igCnt = 0;
    z1 = z2 = z3 = diff = 0; info = 0;
    _filt = new Tp2OrdF();
    _filt->Init(TP_COE_p010);
  }

  void Init()
  {
    // printf("Enc2 init\n");
    ChkFeature2(FEAT_SEM);
    GpIoInit(1ULL << _dirPin, true);
    GpIoInitInterrupt(_intrPin, encoder_isr, this, GPIO_INTR_POSEDGE);
  }

  void ISRFunction()
  {
    checkDir(); // HL - Hack
    _runFlag = 1;
    int64_t t2 = esp_timer_get_time();
    int64_t actPw = t2 - _t1;
    _t1 = t2;

    /* if (frequDiff(pw, actPw) > 100) {
      igCnt++;
      if (igCnt < 3)
        return;
      igCnt = 0;
    } */
    
    info = 0;
    if (IsFrequLow(pw)) {
      info = 200;
      if (Is_F_gt_500(actPw))
        return;
    }
    
    if (actPw < 600) // ignore short pw
      return;
    pw = actPw;
    // checkDir(); HL-Hack
  }
  
  void setBW(int aFiltNum) // 1..small -> 5..big 0..off
  {
    filtOn = true;
    if (aFiltNum < 1 || aFiltNum > 5) {
      filtOn = false; return;
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
      _runFlag = 0; // _filt->Reset();
      pw = 0x0FFFFFF;
    } 
    if (actDir > 0)
      _filt->CalcOneStep((1.0 / (float)pw) * FREQU_FACT);
    else // <=0
      _filt->CalcOneStep((-1.0 / (float)pw) * FREQU_FACT);
    /* if (getFrequUS() < 10) {
      _filt->Reset();
    } */
  }

  void CalcDiff(float aVal)
  {
    diff = aVal - z3;
    z3 = z2; z2 = z1; z1 = aVal;
  }

  // unsigned
  float getFrequUS() 
  {
    return (1.0 / (float)pw) * FREQU_FACT;
  }

  // signed
  float getFrequ() 
  {
    if (actDir > 0)
      return (1.0 / (float)pw) * FREQU_FACT;
    else if (actDir < 0)
      return (-1.0 / (float)pw) * FREQU_FACT;
    else
      return 0.0;
  }

  // filtert
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
    if (pw > 32000)
      return 32000;
    else
      return pw;
  }

  int getDir()
  {
    return actDir;
  }

  void checkDir()
  {
    if (!inv) {
      if (gpio_get_level((gpio_num_t)_dirPin)) {
        cnt++; actDir = 1;
      }
      else {
        cnt--; actDir = -1;
      }
    }
    else {
      if (gpio_get_level((gpio_num_t)_dirPin)) {
        cnt--; actDir = -1;
      }
      else {
        cnt++; actDir = 1;
      }
    }
  }

  int absCnt()
  {
    if ((int)cnt < 0)
      return -(int)cnt;
    return cnt;
  }

private:
  int _dirPin, _intrPin;
  float actDir;
  bool filtOn;
  int64_t _t1;
  uint32_t _runFlag;
  int igCnt;
  Tp2OrdF* _filt;
  float z1, z2, z3;
public:
  int cnt;
  uint32_t pw;
  float diff;
  bool inv;
  int info;
};
