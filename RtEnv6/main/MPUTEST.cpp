
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
  }
}

void DispAcc()
{
  mpu.getAccel();
  ua0.WriteSvI16(1, mpu.getAccelX());
  ua0.WriteSvI16(2, mpu.getAccelY());
  ua0.WriteSvI16(3, mpu.getAccelZ());
}

void DispGyro()
{
  mpu.getGyro();
  ua0.WriteSvI16(1, mpu.getGyroX());
  ua0.WriteSvI16(2, mpu.getGyroY());
  ua0.WriteSvI16(3, mpu.getGyroY());
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
  xTaskCreate(Monitor, "Monitor", 4096, NULL, 10, NULL);
  CommandLoop();
}

