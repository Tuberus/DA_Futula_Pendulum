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
            mot.setPow2(ua0.ReadF());
            ua0.SvMessage("Motor 0.2");
        }
        if (cmd == 3)
        {
            mot.setPow2(0.0f);
            ua0.SvMessage("Stop");
        }
        if(cmd == 4){
            enc.cnt = 0.0f;
            ua0.SvMessage("Enc Reset");
        }
    }
}

extern "C" void Monitor(void *arg)
{
    while (1)
    {
        vTaskDelay(1);
        
        if (ua0.acqON)
        {
            ua0.WriteSvI16(1, motPow);
            ua0.WriteSvI16(2, enc.cnt);
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