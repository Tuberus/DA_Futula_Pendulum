#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "RTEnvHL.h"
#include "Tp1Ord.h"
#include "TimerHL.h"

inline Tp1Ord usTp;

class UsDist {
  gpio_num_t trg, echo;
  StopWatch stw;
public:
  static void IRAM_ATTR dist_isr(void* arg)
  {
    ((UsDist*)arg)->echoISR();
  }

  UsDist(int aTrg, int aEcho)
  {
    trg = (gpio_num_t)aTrg;
    echo = (gpio_num_t)aEcho;
    dist = 0;
  }

  void Init()
  {
    GpIoInit(1ULL << trg, false);
    gpio_set_level(trg, 0);
    GpIoInitInterrupt(echo, dist_isr, this, GPIO_INTR_ANYEDGE);
    usTp.SetAlpha(0.2);
  }

  void setAlpha(float alpha)
  {
    usTp.SetAlpha(alpha);
  }

  void startMeas()
  {
    usTp.CalcOneStep(dist);
    gpio_set_level(trg, 1);
    esp_rom_delay_us(12);
    gpio_set_level(trg, 0);
  }

  void echoISR()
  {
    if (gpio_get_level(echo))
      stw.Reset();
    else
      dist = stw.val();
  }

  float getFilt()
  {
    return usTp.y;
  }

public:
  uint32_t dist;
};
