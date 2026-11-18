#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "MPU_Esp.h"
#include "MotorSetup.h"

SvProtocol3 ua0;
extern MPU6050 mpu;
int dispmode = 1;
bool monOn = true;
volatile float motPow = 0.0f;

void CommandLoop(){
    int cmd;
    while(1){
        cmd = ua0.GetCommand();
        if(cmd == 2){
            motPow = ua0.ReadF();
            mot.setPow2(motPow);
            ua0.SvMessage2("%.2f", motPow);
        }
        if(cmd == 5){
            ua0.SvMessage("RESET");
            mot.setPow2(0.0f);
            enc.cnt = 0.0f;
        }   
    }
}

void Motor_w_MPU(void *args){
    while(1){
        float xw = mpu.getGyroX();
        if(xw <= 0.0f || xw >= 250.0f){
            for(float i = 0; i < 10; i+=0.1f){
                mot.setPow2(i);
            }
        }
        vTaskDelay(1);
    }
}

void Monitor(void *args){
    while(1){
        vTaskDelay(1);

        if(ua0.acqON){
            //WERTE LESEN FUER DIAGRAMM
            ua0.WriteSvI16(1, mpu.getGyroX());
            ua0.WriteSvI16(2, enc.cnt);
        }
    }
}

extern "C" void app_main(void){
    printf("MPU controll w Motor");

    InitRtEnvHL();
    InitMotors();
    I2cInit();
    mpu.Init();
    InitUart(UART_NUM_0, 50000);

    xTaskCreate(Monitor, "Monitor", 4096, NULL, 10, NULL);
    xTaskCreate(Motor_w_MPU, "TESTING", 4096, NULL, 9, NULL);
    CommandLoop();
}
