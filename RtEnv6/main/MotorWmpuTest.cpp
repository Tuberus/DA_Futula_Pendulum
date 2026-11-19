#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "MPU_Esp.h"
#include "KalIMUOff.h"
#include "MotorSetup.h"

SvProtocol3 ua0;
extern MPU6050 mpu;
KalmSimple kalm;

int dispMode = 1;
bool monOn = true;
volatile float motPow = 0.0f;
volatile bool flag = false;

// Achsen an deinen Einbau anpassen
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
            motPow = ua0.ReadF();
            mot.setPow2(motPow);
            ua0.SvMessage("Motor set");
        }
        if (cmd == 3)
        {
            flag = true;
            ua0.SvMessage("START");
        }
        if (cmd == 4)
        {
            int mode = ua0.ReadI16();
            if (mode == 1 || mode == 2)
                dispMode = mode;
            if (mode == 3)
            {
                kalm.Reset();
                ua0.SvMessage("Kalm Reset");
            }
        }
        if (cmd == 5)
        {
            flag = false; // zuerst Flag, dann Motor
            mot.setPow2(0.0f);
            enc.cnt = 0.0f;
            kalm.Reset();
            ua0.SvMessage("RESET");
        }
    }
}

void Motor_w_MPU(void *args)
{
    constexpr float KP = 0.02f; // Platzhalter, am Aufbau einstellen
    while (1)
    {
        if (flag)
        {
            float angle = kalm.complAngle; // Winkel aus dem Filter
            float out = KP * angle;
            if (out > 1.0f)
                out = 1.0f; // an Wertebereich von setPow2 anpassen
            if (out < -1.0f)
                out = -1.0f;
            mot.setPow2(out);
        }
        vTaskDelay(1);
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
    ua0.WriteSvI16(3, enc.cnt);
}

extern "C" void Monitor(void *args)
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
    printf("MPU control w Motor\n");

    InitRtEnvHL();
    InitMotors();
    I2cInit();
    mpu.Init();
    InitUart(UART_NUM_0, 500000);

    gyroCh.CalGyro(); // Board dabei ruhig halten!

    xTaskCreate(Monitor, "Monitor", 4096, NULL, 10, NULL);
    xTaskCreate(ImuTask, "IMU", 4096, NULL, 20, NULL);
    xTaskCreate(Motor_w_MPU, "Motor", 4096, NULL, 9, NULL);
    CommandLoop();
}