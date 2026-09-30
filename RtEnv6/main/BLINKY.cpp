#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"

#define LED_PIN 2

SvProtocol3 ua0;

GpIoOut LED(LED_PIN);

extern "C" void BlinkTask(void* args){
    while(1){
        vTaskDelay(2);
        LED.Toggle();
    }
}


void CommandLoop()
{
  int cmd;
  while (1) {
    cmd = ua0.GetCommand();
  }
}


extern "C" void app_main(void){
    printf("Blinky\n");

    InitRtEnvHL();
    LED.Init();

    InitUart(UART_NUM_0, 50000);

    xTaskCreate(BlinkTask, "Blinky", 2048, NULL, 10, NULL);
    CommandLoop();
}

