#pragma once

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "MPU_Regs.h"
#include "Features.h"

#ifndef I2C_TIM_O
#define I2C_TIM_O (1000/portTICK_PERIOD_MS)
#endif

inline void I2cDelay()
{
  vTaskDelay(10);
}

inline void I2cInit()
{
  ChkFeature2(FEAT_SEM);
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = 21;
  conf.scl_io_num = 22;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 400000;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;
  i2c_param_config(I2C_NUM_0, &conf);
  ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
}

inline esp_err_t RegRead(uint8_t reg_addr, void* data, size_t len)
{
  return i2c_master_write_read_device
  (I2C_NUM_0, MPU_ADDRESS, &reg_addr, 1, (uint8_t*)data, len, I2C_TIM_O);
}

inline char RegReadB(uint8_t reg_addr)
{
  char retval;
  ESP_ERROR_CHECK(RegRead(reg_addr, &retval, 1));
  return retval;
}

inline esp_err_t RegWriteB(uint8_t reg_addr, uint8_t data)
{
  int ret;
  uint8_t write_buf[2] = { reg_addr, data };
  ret = i2c_master_write_to_device(I2C_NUM_0, MPU_ADDRESS, write_buf, sizeof(write_buf), I2C_TIM_O);
  return ret;
}

class MPU6050 {
public:
  int acc[3];   // acc raw vals
  int gyro[3];  // gyro raw vals
public:
  MPU6050()
  {
    for (int i = 0; i < 3; i++)
      acc[i] = gyro[i] = 0;
  }
  
  int testConnection()
  {
    ChkFeature2(FEAT_SEM);
    char temp;
    temp = RegReadB(MPU_WHO_AM_I_REG);
    // ESP_ERROR_CHECK(temp==0x68);
    return temp;
  }

  void Init()
  {
    ChkFeature2(FEAT_SEM);
    I2cDelay(); setSleepMode(false);
    I2cDelay(); setGyroRange(MPU_GYRO_RANGE_2000);
    I2cDelay(); setAccelRange(MPU_ACCELERO_RANGE_16G);
    I2cDelay(); setBW(MPU_BW_256);
    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips 
    // can join the I2C bus and all can be controlled by the Arduino as master
    // write(INT_PIN_CFG, 0x22);
    // write(INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  }

  void setBW(char BW)
  {
    char temp;
    BW = BW & 0x07;
    temp = RegReadB(MPU_CONFIG_REG);
    temp &= 0xF8;
    temp = temp + BW;
    RegWriteB(MPU_CONFIG_REG, temp);
  }

  void setI2CBypass(bool state)
  {
    char temp;
    temp = RegReadB(MPU_INT_PIN_CFG);
    if (state == true)
      temp |= 1 << MPU_BYPASS_BIT;
    if (state == false)
      temp &= ~(1 << MPU_BYPASS_BIT);
    RegWriteB(MPU_INT_PIN_CFG, temp);
  }

  void setAccelRange(char range)
  {
    char temp;
    range = range & 0x03;
    temp = RegReadB(MPU_ACCELERO_CONFIG_REG);
    temp &= ~(3 << 3);
    temp = temp + (range << 3);
    RegWriteB(MPU_ACCELERO_CONFIG_REG, temp);
  }
  
  int getAccelX()
  {
    short retval;
    char data[2];
    ESP_ERROR_CHECK(RegRead(MPU_ACCEL_XOUT_H_REG, data, 2));
    retval = (data[0] << 8) + data[1];
    acc[0] = retval;
    return (int)retval;
  }

  int getAccelY()
  {
    short retval;
    char data[2];
    ESP_ERROR_CHECK(RegRead(MPU_ACCEL_YOUT_H_REG, data, 2));
    retval = (data[0] << 8) + data[1];
    acc[1] = retval;
    return (int)retval;
  }

  int getAccelZ()
  {
    short retval;
    char data[2];
    ESP_ERROR_CHECK(RegRead(MPU_ACCEL_ZOUT_H_REG, data, 2));
    retval = (data[0] << 8) + data[1];
    acc[2] = retval;
    return (int)retval;
  }

  void getAccel()
  {
    char temp[6];
    ESP_ERROR_CHECK(RegRead(MPU_ACCEL_XOUT_H_REG, temp, 6));
    acc[0] = (int)(short)((temp[0] << 8) + temp[1]);
    acc[1] = (int)(short)((temp[2] << 8) + temp[3]);
    acc[2] = (int)(short)((temp[4] << 8) + temp[5]);
  }

  void setGyroRange(char range)
  {
    char temp;
    range = range & 0x03;
    temp = RegReadB(MPU_GYRO_CONFIG_REG);
    temp &= ~(3 << 3);
    temp = temp + (range << 3);
    RegWriteB(MPU_GYRO_CONFIG_REG, temp);
  }

  int getGyroX()
  {
    short retval;
    char data[2];
    ESP_ERROR_CHECK(RegRead(MPU_GYRO_XOUT_H_REG, data, 2));
    retval = (data[0] << 8) + data[1];
    gyro[0] = retval;
    return (int)retval;
  }

  int getGyroY()
  {
    short retval;
    char data[2];
    ESP_ERROR_CHECK(RegRead(MPU_GYRO_YOUT_H_REG, data, 2));
    retval = (data[0] << 8) + data[1];
    gyro[1] = retval;
    return (int)retval;
  }

  int getGyroZ()
  {
    short retval;
    char data[2];
    ESP_ERROR_CHECK(RegRead(MPU_GYRO_ZOUT_H_REG, data, 2));
    retval = (data[0] << 8) + data[1];
    gyro[2] = retval;
    return (int)retval;
  }

  void getGyro()
  {
    char temp[6];
    ESP_ERROR_CHECK(RegRead(MPU_GYRO_XOUT_H_REG, temp, 6));
    gyro[0] = (int)(short)((temp[0] << 8) + temp[1]);
    gyro[1] = (int)(short)((temp[2] << 8) + temp[3]);
    gyro[2] = (int)(short)((temp[4] << 8) + temp[5]);
  }

  int getTempRaw(void)
  {
    short retval;
    char data[2];
    ESP_ERROR_CHECK(RegRead(MPU_TEMP_H_REG, data, 2));
    retval = (data[0] << 8) + data[1];
    return (int)retval;
  }

  float getTemp(void)
  {
    float retval;
    retval = (float)this->getTempRaw();
    retval = (retval + 521.0) / 340.0 + 35.0;
    return retval;
  }
  
  int CheckReadyBit()
  {
    return RegReadB(INT_STATUS) & 0x01;
  }
  
  void setSleepMode(bool state)
  {
    char temp;
    temp = RegReadB(MPU_PWR_MGMT_1_REG);
    if (state == true)
      temp |= 1 << MPU_SLP_BIT;
    if (state == false)
      temp &= ~(1 << MPU_SLP_BIT);
    RegWriteB(MPU_PWR_MGMT_1_REG, temp);
  }
};

inline MPU6050 mpu;

inline void initMPU() // calls I2cInit()
{
  I2cInit(); 
  printf("Conn: %X\n", mpu.testConnection()); 
  mpu.Init();
}
