#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "MPU_Esp.h"
#include "KaliMUOff.h"
#include "MotorSetup.h"

SvProtocol3 ua0;
extern MPU6050 mpu;
KalmSimple kalm;

int dispMode = 2;
bool monOn = true;

constexpr int ACC_AXIS = x_idx;
constexpr int GYRO_AXIS = y_idx;

ImuChan accCh(mpu.acc, ImuChan::Type::Acc);
ImuChan gyroCh(mpu.gyro, ImuChan::Type::Gyro);

void CommandLoop()
{
    int cmd;
    while (1)
    {
        cmd = ua0.GetCommand();
        if (cmd == 2)
        {
            int mode = ua0.ReadI16();
            if (mode == 1 || mode == 2)
                dispMode = mode;
            if (mode == 1)
                ua0.SvMessage("Disp Angle");
            if (mode == 2)
                ua0.SvMessage("Disp Gyro");
            if (mode == 3)
            {
                kalm.Reset();
                ua0.SvMessage("Kalm Reset");
            }
        }
    }
}

void DispAngle()
{
    ua0.WriteSvI16(1, kalm.accAngle);
    ua0.WriteSvI16(2, kalm.gySum);
    ua0.WriteSvI16(3, kalm.complAngle);
}

void DispGyro()
{
    ua0.WriteSvI16(1, kalm.omega);
    ua0.WriteSvI16(2, gyroCh.getVal(GYRO_AXIS));
    ua0.WriteSvI16(3, accCh.getFilt1(ACC_AXIS));
}

extern "C" void Monitor(void *arg)
{
    while (1)
    {
        vTaskDelay(1);
        if (ua0.acqON && monOn)
        {
            if (dispMode == 1)
                DispAngle();
            if (dispMode == 2)
                DispGyro();
            ua0.Flush();
        }
    }
}

extern "C" void ImuTask(void *args)
{
    while (1)
    {
        mpu.getAccel();
        mpu.getGyro();
        accCh.CalcFilt(ACC_AXIS);
        kalm.CalcFilter(accCh.getFilt2(ACC_AXIS),
                        gyroCh.getVal(GYRO_AXIS));
    }
}

extern "C" void app_main(void)
{
    printf("MpuTest1_1\n");
    InitRtEnvHL();
    I2cInit();

    mpu.Init();
    InitUart(UART_NUM_0, 500000);

    gyroCh.CalGyro();

    xTaskCreate(Monitor, "Monitor", 4096, NULL, 10, NULL);
    xTaskCreate(ImuTask, "IMU", 4096, NULL, 20, NULL);
    CommandLoop();
}