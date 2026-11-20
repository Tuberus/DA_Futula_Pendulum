#include <cstdio>
#include <cmath>
#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "MPU_Esp.h"
#include "KalIMUOff.h"
#include "MotorSetup.h"
#include "PID.h"

SvProtocol3 ua0;
extern MPU6050 mpu;
KalmSimple kalm;
PendulumPID pid;

int dispMode = 3;
bool monOn = true;
volatile bool ctrlOn = false;
volatile float motorOut = 0.0f;
volatile float manualOut = 0.0f;
volatile float armAngleDeg = 0.0f;

constexpr int ACC_AXIS = x_idx;
constexpr int GYRO_AXIS = y_idx;
constexpr bool MOTOR_INV = false;
constexpr float DT = portTICK_PERIOD_MS / 1000.0f;
constexpr float TICKS_PER_REV = 330.0f;
constexpr int MAX_ENC_TICKS = 119;
constexpr int MIN_ENC_TICKS = -161;
constexpr int ENC_BUFFER = 15;

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
            if (mode >= 1 && mode <= 4)
                dispMode = mode;
            if (mode == 1)
                ua0.SvMessage("Disp Angle");
            if (mode == 2)
                ua0.SvMessage("Disp Gyro");
            if (mode == 3)
                ua0.SvMessage("Disp PID");
            if (mode == 4)
                ua0.SvMessage("Disp ENC");
            if (mode == 5)
            {
                kalm.Reset();
                ua0.SvMessage("Kalm Reset");
            }
        }
        else if (cmd == 3)
        {
            manualOut = 0.0f;
            ctrlOn = (ua0.ReadI16() != 0);
            pid.reset();
            ua0.SvMessage(ctrlOn ? "PID EIN" : "PID AUS");
        }
        else if (cmd == 4)
        {
            pid.setpoint = kalm.complAngle;
            pid.reset();
            enc.cnt = 0;
            armAngleDeg = 0.0f;
            char msg[48];
            snprintf(msg, sizeof(msg), "Nullpunkt gesetzt: %.2f", pid.setpoint);
            ua0.SvMessage(msg);
        }
        else if (cmd == 5)
        {
            pid.setpoint = ua0.ReadF();
            ua0.SvMessage("Sollwert manuell gesetzt");
        }
        else if (cmd == 6)
        {
            pid.kp = ua0.ReadF();
            pid.ki = ua0.ReadF();
            pid.kd = ua0.ReadF();
            ua0.SvMessage("PID Werte aktualisiert");
        }
        else if (cmd == 7)
        {
            ctrlOn = false;
            manualOut = ua0.ReadF();
            ua0.SvMessage("Manual Power gesetzt");
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

void DispEnc()
{
    ua0.WriteSvI16(1, (int16_t)enc.cnt);
    ua0.WriteSvI16(2, (int16_t)armAngleDeg);
}

void DispPid()
{
    ua0.WriteSvI16(1, (int16_t)kalm.complAngle);
    ua0.WriteSvI16(2, (int16_t)pid.setpoint);
    ua0.WriteSvI16(3, (int16_t)(motorOut * 100.0f));
    ua0.WriteSvI16(4, (int16_t)enc.cnt);
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
            if (dispMode == 3)
                DispPid();
            if (dispMode == 4)
                DispEnc();
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

        armAngleDeg = enc.cnt * (360.0f / TICKS_PER_REV);

        if (ctrlOn)
        {
            if (enc.cnt > (MAX_ENC_TICKS - ENC_BUFFER) ||
                enc.cnt < (MIN_ENC_TICKS + ENC_BUFFER))
            {
                ctrlOn = false;
                motorOut = 0.0f;
                pid.reset();
                ua0.SvMessage("STOP: Encoder Limit erreicht");
            }
            else
            {
                motorOut = pid.compute(kalm.complAngle, DT);
            }
        }
        else
        {
            motorOut = manualOut;
            pid.reset();
        }

        mot.setPow(MOTOR_INV ? -motorOut : motorOut);
        vTaskDelay(1);
    }
}

extern "C" void app_main(void)
{
    printf("PendulumCtrl\n");
    InitRtEnvHL();
    I2cInit();

    mpu.Init();
    InitMotors();
    InitUart(UART_NUM_0, 500000);

    pid.outMax = 0.5f;
    pid.kp = 0.005f;
    pid.ki = 0.0f;
    pid.kd = 0.0f;

    gyroCh.CalGyro();

    xTaskCreate(Monitor, "Monitor", 4096, NULL, 10, NULL);
    xTaskCreate(ImuTask, "IMU", 4096, NULL, 20, NULL);
    CommandLoop();
}