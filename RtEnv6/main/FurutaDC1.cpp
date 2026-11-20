#include <cstdio>
#include <cmath>
#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "MPU_Esp.h"
#include "KalIMUOff.h"
#include "Furuta.h"

SvProtocol3 ua0;

FurutaController balC(2E-2, 0, 0, 0, "BalC");

constexpr int ACC_AXIS = x_idx;
constexpr int GYRO_AXIS = y_idx;
constexpr bool MOTOR_INV = false;

constexpr int MAX_ENC_TICKS = 104;
constexpr int MIN_ENC_TICKS = -146;

void DoDisplay();

void ControlParams(int cmd)
{
  if (cmd == 200) {
    int num = ua0.ReadI16();
    (void)num;
    balC.ReadCOM(&ua0);
    ua0.SvMessage("Set PID");
  }
  if (cmd == 201) {
    int num = ua0.ReadI16();
    (void)num;
    ua0.LockOStream();
      ua0.WrB(200);
      balC.WriteCOM(&ua0);
      ua0.Flush();
    ua0.UnlockOStream();
    ua0.SvMessage("Get PID");
  }
}

void CommandLoop()
{
  int cmd;
  while (1) {
    cmd = ua0.GetCommand();
    if (cmd == 2) {
      float pow = ua0.ReadF();
      motL.setPow(MOTOR_INV ? -pow : pow);
      ua0.SvMessage("Set Pow");
    }
    if (cmd == 3) {
      encL.cnt = 0;
      ua0.SvMessage("Reset Enc");
    }
    if (cmd == 4) {
      balC.on = ua0.ReadI16();
      if (!balC.on) {
        vTaskDelay(2);
        motL.setPow(0.0f);
      }
      ua0.SvMessage2("RGL", balC.on);
    }
    if (cmd == 5) {
      balC.demPos = ua0.ReadI16();
      ua0.SvMessage("Set Pos");
    }
    if (cmd == 6) {
      calFlag = true;
      ua0.SvMessage("Cal start");
      imuGyro.CalGyro();
      kalm.Reset();
      calFlag = false;
      ua0.SvMessage("Cal done!!");
    }
    if (cmd == 7) {
      int val = ua0.ReadI16();
      encL.setBW(val);
      ua0.SvPrintf("BW: %d", val);
    }
    if (cmd == 8) {
      float offs = ua0.ReadF();
      if (offs != -1)
        balC.angleOffs = offs;
      ua0.SvPrintf("angleOffs: %1.1f", balC.angleOffs);
    }
    if (cmd == 10) {
      balC.angleOffs = -kalm.complAngle;
      encL.cnt = 0;
      balC.demPos = 0;
      ua0.SvPrintf("Nullpunkt Offs: %1.1f", balC.angleOffs);
    }
    if (cmd == 200 || cmd == 201)
      ControlParams(cmd);
  }
}

extern "C" void RPM_Task(void* arg)
{
  encL.CalcFilt2();
  if (encL.cnt > MAX_ENC_TICKS || encL.cnt < MIN_ENC_TICKS)
    motL.setPow(0.0f);
  DoDisplay();
}

extern "C" void RglTask(void* arg)
{
  if (calFlag)
    return;

  mpu.getAccel();
  mpu.getGyro();
  imuAcc.CalcFilt(ACC_AXIS);
  kalm.CalcFilter(imuAcc.getFilt2(ACC_AXIS), imuGyro.getVal(GYRO_AXIS));

  if (encL.cnt > MAX_ENC_TICKS || encL.cnt < MIN_ENC_TICKS) {
    balC.on = 0;
    motL.setPow(0.0f);
    return;
  }
  if (!balC.on)
    return;

  balC.CalcOneStep(kalm.complAngle, encL.getFrequF(), encL.cnt);

  if (std::fabs(kalm.complAngle + balC.angleOffs) < 300.0f) {
    motL.setPow(MOTOR_INV ? -balC.pow : balC.pow);
  } else {
    balC.on = 0;
    motL.setPow(0.0f);
  }
}

void DoDisplay()
{
  if (ua0.acqON) {
    ua0.LockOStream();
    ua0.WriteSvI16(1, encL.cnt);
    ua0.WriteSvF(2, encL.getFrequ());
    ua0.WriteSvI16(3, kalm.complAngle);
    ua0.Flush();
    ua0.UnlockOStream();
  }
}

extern "C" void app_main(void)
{
  printf("FurutaDC1\n");
  InitRtEnvHL();
  I2cInit();
  printf("Conn: %X\n", mpu.testConnection());
  mpu.Init();
  InitMotors();
  InitUart(UART_NUM_0, 500000);
  imuGyro.CalGyro();

  EspTimSetup(300, RPM_Task, 0, false);
  EspTimSetup(1000, RglTask, 0, false);
  CommandLoop();
}