#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "MotorSetup.h"

SvProtocol3 ua0;

volatile float motPow = 0.0f;

void CommandLoop()
{
    int cmd;
    while (1)
    {
        cmd = ua0.GetCommand();
        if (cmd == 2)
        {
            motPow = 0.5f; // Oder ua0.ReadF()
            mot.setPow2(motPow);
            ua0.SvMessage("Test geschw");
        }
        if (cmd == 3)
        {
            motPow = 0.0f;
            mot.setPow2(0.0f);
            ua0.SvMessage("Stop");
        }
    }
}

extern "C" void Monitor(void *arg)
{
    while (1)
    {
        vTaskDelay(1);
        
        enc.CalcDiff(); 

        if (ua0.acqON)
        {
            ua0.WriteSvI16(1, motPow * 1000);
            ua0.WriteSvI16(2, enc.getFrequ());
            ua0.Flush();
        }
    }
}

extern "C" void app_main(void)
{
    printf("MotorTest\n");
    InitRtEnvHL();
    InitMotors();
    InitUart(UART_NUM_0, 500000);
    xTaskCreate(Monitor, "Monitor", 2048, NULL, 10, NULL);
    CommandLoop();
}