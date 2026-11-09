#pragma once

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/dac_oneshot.h"
#include "TimerHL.h"
#include "Features.h"
#include "FloatFilt.h"
#include "NodeLock.h"
#include "SvProtocol3.h"

inline void LibVers()
{
  printf("RtEnvHL V2.1\n");
}

inline void MyDelay(int aMSec)
{
  vTaskDelay(aMSec / portTICK_PERIOD_MS);
}

inline void InitRtEnvHL([[maybe_unused]] bool doBlink=false)
{
  LibVers();
  SvProtocol3::Init();
  InitNVS();
  openRtEnvStore();
  CheckNodeLock();
  closeRtEnvStore();
}

// core=-1 any core
inline TaskHandle_t createTask(TaskFunction_t func,
                               const char* name, int stackSz, int prio, int core=-1)
{
  TaskHandle_t han;
  if( core==-1 )
    xTaskCreate(func, name, stackSz, NULL, prio, &han);
  else
    xTaskCreatePinnedToCore(func, name, stackSz, NULL, prio, &han, core);
  return han;
}

inline void GpIoInit(uint64_t aBitMask, bool aIn=false)
{
  ChkFeature2(FEAT_SEM);
  gpio_config_t conf = {};
  conf.pin_bit_mask = aBitMask;
  conf.mode = GPIO_MODE_OUTPUT;
  conf.pull_up_en = GPIO_PULLUP_DISABLE;
  conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  conf.intr_type = GPIO_INTR_DISABLE;
  if (aIn) {
    conf.mode = GPIO_MODE_INPUT;
    conf.pull_up_en = GPIO_PULLUP_ENABLE;
  }
  ESP_ERROR_CHECK(gpio_config(&conf));
}

inline void GpIoDir(gpio_num_t aNum, gpio_mode_t aMode)
{
  gpio_set_direction(aNum, aMode);
}

inline void GpIoLevel(gpio_num_t aNum, uint32_t aVal)
{
  gpio_set_level(aNum, aVal);
}

class GpIoOut {
public:
  uint16_t pin;
  uint16_t val;
public:
  GpIoOut(int aPin)
  {
    pin = aPin; val = 0;
  }

  void Init()
  {
    GpIoInit(1ULL<<pin, false);
  }

  void Set(int aVal)
  {
    val = aVal; gpio_set_level((gpio_num_t)pin,val);
  }

  void Toggle()
  {
    if (val) Set(0);
    else Set(1);
  }
};


class GpIoIn {
public:
  uint32_t pin;
public:
  GpIoIn(int aPin)
  {
    pin = aPin;
  }
  void Init()
  {
    GpIoInit(1ULL<<pin, true);
  }
  int get()
  {
    return gpio_get_level((gpio_num_t)pin);
  }
};


class StopWatch {
public:
  int64_t dt;
public:
  StopWatch()
  {
    _t1 = dt = 0; 
  }
  void Reset()
  {
    _t1 = esp_timer_get_time();
  }
  int64_t val()
  {
    dt = esp_timer_get_time() - _t1;
    return dt;
  }
private:
  int64_t _t1;
};


class Adc2 {
private:
  adc_channel_t _ch;
  inline static adc_oneshot_unit_handle_t _unit2 = 0;
public:
  inline static adc_atten_t atten = ADC_ATTEN_DB_11;
public:
  Adc2(adc_channel_t aChan)
  {
    _ch = aChan;
  }
  void Init()
  {
    if (_unit2 == 0) {
      adc_oneshot_unit_init_cfg_t inicfg = {};
      inicfg.unit_id = ADC_UNIT_2;
      ESP_ERROR_CHECK(adc_oneshot_new_unit(&inicfg, &_unit2));
    }
    adc_oneshot_chan_cfg_t cfg2 = {};
    cfg2.atten = Adc2::atten;
    cfg2.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_unit2, _ch, &cfg2));
  }
  int read()
  {
    int val;
    adc_oneshot_read(_unit2, _ch, &val);
    return val;
  }
};

class Adc1 {
public:
  adc_channel_t _ch;
  inline static adc_oneshot_unit_handle_t _unit1 = 0;
public:
  inline static adc_atten_t atten = ADC_ATTEN_DB_11;
public:
  Adc1(adc_channel_t aChan)
  {
    _ch = aChan;
  }
  Adc1()
  {
    _ch = (adc_channel_t)0;
  }
  void Init()
  {
    if (_unit1 == 0) {
      adc_oneshot_unit_init_cfg_t inicfg = {};
      inicfg.unit_id = ADC_UNIT_1;
      ESP_ERROR_CHECK(adc_oneshot_new_unit(&inicfg, &_unit1));
    }
    adc_oneshot_chan_cfg_t cfg2 = {};
    cfg2.atten = Adc1::atten;
    cfg2.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_unit1, _ch, &cfg2));
  }
  int read()
  {
    int val;
    adc_oneshot_read(_unit1, _ch, &val);
    return val;
  }
};


class Dac {
public:
  Dac(dac_channel_t aChan)
  {
    _ch = aChan;
  }
  void Init()
  {
    dac_oneshot_config_t cfg = {};
    cfg.chan_id = _ch;
    ESP_ERROR_CHECK(dac_oneshot_new_channel(&cfg, &_hdac));
  }
  void write(int aVal)
  {
    dac_oneshot_output_voltage(_hdac, aVal);
  }
  void writeF(float aVal)
  {
    uint8_t dval = (aVal * 0.5 + 0.5) * 256.0;
    dac_oneshot_output_voltage(_hdac, dval);
  }
private:
  dac_channel_t _ch;
  dac_oneshot_handle_t _hdac;
};

