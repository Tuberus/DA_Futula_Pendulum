#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "TimerHL.h"

class ButtonHandler {
private:
  int _pin;
  SemaphoreHandle_t _sema;
  bool _waiting;
public:
  static void IRAM_ATTR button_isr(void* arg)
  {
    ((ButtonHandler*)arg)->ISRFunction();
  }

  ButtonHandler(int aPin)
  {
    _pin = aPin;
    cnt = 0;
    _waiting = false;
    _sema = nullptr;
  }

  void Init()
  {
    GpIoInitInterrupt(_pin, button_isr, this, GPIO_INTR_POSEDGE);
    _sema = xSemaphoreCreateBinary();
  }

  void ISRFunction()
  {
    if (check() != 1)
      return;
    if (_waiting) {
      _waiting = false;
      BaseType_t prioThings = pdFALSE;
      xSemaphoreGiveFromISR(_sema, &prioThings);
      portYIELD_FROM_ISR(prioThings);
    } 
    cnt++;
  }

  void wait()
  {
    while (1) {
      _waiting = true;
      xSemaphoreTake(_sema, portMAX_DELAY);
      vTaskDelay(10);
      if (check() == 1)
        return;
    }
  }

  int check()
  {
    return gpio_get_level((gpio_num_t)_pin);
  }

  void listen()
  {
    cnt = 0;
  }

  bool wasClicked()
  {
    return cnt != 0;
  }

  void waitBtnRelease()
  {
    while (check())
      vTaskDelay(1);
  }

public:
  int cnt;
};
