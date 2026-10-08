
#include "RTEnvHL.h"
#include "SvProtocol3.h"
#include "EspMotor.h"
#include "MPU_Esp.h"

SvProtocol3 ua0;
extern MPU6050 mpu;
int dispMode = 1;
bool monOn = true;

void CommandLoop()
{
  // printf("CommandLoop\n");
  int cmd;
  while (1) {
    cmd = ua0.GetCommand();
    if (cmd == 2) {
      dispMode = ua0.ReadI16();
      if (dispMode == 1)
        ua0.SvMessage("Disp Acc");
      if (dispMode == 2)
        ua0.SvMessage("Disp Gyro");
    }
    if (cmd == 3) {
      monOn = false;
       // 0:2g .... 3:16g
       mpu.setAccelRange(ua0.ReadI16());
      monOn = true;
      ua0.SvMessage("acc range");
    }
    else if (cmd == 50) {
      esp_restart();
    }
  }
}

void DispAcc()
{
  mpu.getAccel();
  printf("Acc: X=%d, Y=%d, Z=%d\n", mpu.acc[0], mpu.acc[1], mpu.acc[2]);
  ua0.WriteSvI16(1, mpu.acc[0]);
  ua0.WriteSvI16(2, mpu.acc[1]);
  ua0.WriteSvI16(3, mpu.acc[2]);
}

void DispGyro()
{
  mpu.getGyro();
  ua0.WriteSvI16(1, mpu.gyro[0]);
  ua0.WriteSvI16(2, mpu.gyro[1]);
  ua0.WriteSvI16(3, mpu.gyro[2]);
}

extern "C" void Monitor(void* arg)
{
  while (1) {
    vTaskDelay(1);
    if (ua0.acqON && monOn ) {
      if (dispMode == 1)
        DispAcc();
      if (dispMode == 2)
        DispGyro();
      ua0.Flush();
    }
  }
}

extern "C" void app_main(void)
{
  printf("MpuTest1_1\n");
  InitRtEnvHL();
  I2cInit(); 
  
  if (mpu.testConnection()) {
      printf("MPU6050 verbunden!\n");
  } else {
      printf("Warnung: MPU6050 antwortet nicht auf I2C!\n");
  }
  
  mpu.Init();
  InitUart(UART_NUM_0, 500000);
  xTaskCreate(Monitor, "Monitor", 2048, NULL, 10, NULL);
  CommandLoop();
}

