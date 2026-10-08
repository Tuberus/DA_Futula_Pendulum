#include <stdio.h>
#include "Features.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "nvs_flash.h"
#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "driver/gpio.h"
#include "MPU_Esp.h"
#include "ImuAlgo.h"
#include "NodeLock.h"

SvProtocol3* ua0 = nullptr;  
extern MPU6050 mpu;

extern void InitNVS();
extern void openRtEnvStore();
extern void closeRtEnvStore();
extern void Mac2NVS(uint32_t aFeatures);
extern void CheckNodeLock();

void CommandLoop()
{
    while (1) {
        int cmd = ua0->GetCommand();
        if (cmd == 2) { ua0->SvMessage("cal start"); ua0->Flush(); }
        else if (cmd == 3) { ua0->SvMessage("reset"); ua0->Flush(); }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main(void)
{
    printf("SvVis Start\n");

    InitNVS();
    openRtEnvStore();
    Mac2NVS(0x8000007F);   
    CheckNodeLock();
    closeRtEnvStore();

    ua0 = new SvProtocol3();        

    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 2048, 2048, 0, NULL, 0);

    initMPU();

    xTaskCreate(Monitor, "Monitor", 4096, NULL, 5, NULL);
    CommandLoop();
}