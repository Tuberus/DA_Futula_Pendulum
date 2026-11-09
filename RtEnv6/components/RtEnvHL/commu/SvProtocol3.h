#pragma once

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "lwip/sockets.h"
#include "NodeLock.h"
#include "Features.h"

void MyDelay(int aMSec);

// #define USE_UDP 

extern "C" void print_task_list();

extern "C" void wifi_disconnect_impl();

inline void wifi_disconnect()
{
#ifdef USE_UDP
  wifi_disconnect_impl();
  #warning "!!USE_UDP!!"
#endif
}

#define UA_BUF_SIZE (1024)

#ifdef USE_UDP
inline struct sockaddr_storage g_source_addr;
#define SOCK_LEN2  (sizeof(struct sockaddr_storage))
inline socklen_t g_socklen = SOCK_LEN2;
#endif
inline int svSock = 0;
inline uart_port_t gUaNum = UART_NUM_0;
inline SemaphoreHandle_t sem2 = 0;

inline void InitUart(uart_port_t uaNum, int aBaud)
{
  ChkFeature2(FEAT_SEM);
  vTaskDelay(10);
  gUaNum = uaNum;
  uart_config_t uart_config = {};
  uart_config.baud_rate = aBaud;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uart_config.source_clk = UART_SCLK_DEFAULT;

  const int intr_alloc_flags = 0;
  // const int intr_alloc_flags = ESP_INTR_FLAG_IRAM;
  ESP_ERROR_CHECK(uart_driver_install(uaNum, UA_BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
  ESP_ERROR_CHECK(uart_param_config(uaNum, &uart_config));
  if (uaNum == UART_NUM_0) {
    ESP_ERROR_CHECK(uart_set_pin(uaNum, 1, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  }
  if (uaNum == UART_NUM_2) {  // S2
    ESP_ERROR_CHECK(uart_set_pin(uaNum, 17, 16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
  }
  vTaskDelay(10);
}

class UdpRd {
public:
  int rdLen;
private:
  static const int BUFF_SZ = 100;
  uint8_t buff[BUFF_SZ];
  int rdIdx;
public:
  void ReadMsg()
  {
    rdIdx = 0;
#ifdef USE_UDP
    rdLen=recvfrom(svSock, buff, BUFF_SZ-2, 0, (struct sockaddr*)&g_source_addr, &g_socklen);
#endif
  }

  uint8_t ReadByte()
  {
    uint8_t val = buff[rdIdx];
    rdIdx++;
    return val;
  }

  int16_t ReadI16()
  {
    int16_t* ptr = (int16_t*)&buff[rdIdx];
    rdIdx += 2;
    return *ptr;
  }

  void ReadI16Ary(int16_t* ary, int len)
  {
    int16_t* ptr = (int16_t*)&buff[rdIdx];
    for (int i = 0; i < len; i++)
      ary[i] = *ptr++;
    rdIdx += 2 * len;
  }

  float ReadF()
  {
    float* ptr = (float*)&buff[rdIdx];
    rdIdx += 4;
    return *ptr;
  }
};


class SvProtocol3 {
private:
  static const int BUFF_SZ = 256;
  static const int N_PRINT_BUFF = 128;
  uint8_t buff[BUFF_SZ];
  int idx;
  int roundCnt;
  uint8_t resetFlag;
#ifdef USE_UDP
  UdpRd udp;
#endif 
public:
  uint8_t acqON;
  int sendTim;
  int maxSendTim;
public:
  SvProtocol3()
  {
    idx = acqON = 0;
    resetFlag = 1;
    sendTim = maxSendTim = roundCnt = 0;
  }

  static void Init()
  {
    sem2 = xSemaphoreCreateMutex();
  }

  void LockOStream()
  {
    xSemaphoreTake(sem2, portMAX_DELAY);
  }

  void UnlockOStream()
  {
    xSemaphoreGive(sem2);
  }

  void WriteIO(void* aData, size_t aLen)
  {
#ifdef USE_UDP
    int64_t t1 = esp_timer_get_time();
    sendto(svSock, aData, aLen, 0, (struct sockaddr*)&g_source_addr, SOCK_LEN2);
    sendTim = esp_timer_get_time() - t1;
    if (sendTim > maxSendTim)
      maxSendTim = sendTim;
#else
    uart_write_bytes(gUaNum, aData, aLen);
#endif
  }

  int ReadIO(void* aBuff, uint32_t aLen)
  {
#ifdef USE_UDP
    return recvfrom(svSock, aBuff, aLen, 0, (struct sockaddr*)&g_source_addr, &g_socklen);
#else
    return uart_read_bytes(gUaNum, aBuff, aLen, portMAX_DELAY);
#endif
  }

  void Flush()
  {
    WriteIO(buff, idx); idx = 0;
  }

  void Flush2(int aNumBytes)
  {
    if (idx >= aNumBytes)
      Flush();
  }

  void Flush3(int aNumRounds)
  {
    roundCnt++;
    if (roundCnt >= aNumRounds) {
      WriteIO(buff, idx); idx = 0;
      roundCnt = 0;
    }
  }

  void Reset()
  {
    idx = 0;
  }

  int GetCommand()
  {
#ifdef USE_UDP
    udp.ReadMsg();
    uint8_t cmd = udp.ReadByte();
#else
    uint8_t cmd = ReadByte();
#endif
    if (cmd == 1) {
      acqON = ReadByte();
      if (resetFlag) {
        resetFlag = 0;
        SvMessage("Reset!!");
      }
      if (acqON)
        SvMessage("AcqON");
      else
        SvMessage("AcqOFF");
      return 0;
    }
    if (cmd == 51) {
      WrF(1.23456); // NodeLock Token
      Flush();
      return 0;
    }
    return cmd;
  }

  void SvPrintf(const char* format, ...)
  {
    char locBuff[N_PRINT_BUFF];
    char* ptr = locBuff;
    *ptr = 10; ptr++;
    va_list vArgs;
    va_start(vArgs, format);
    vsprintf(ptr, (char const*)format, vArgs);
    va_end(vArgs);
    int len = strlen(locBuff); locBuff[len] = '\n';
    WriteIO(locBuff, len + 1);
  }

  void SvMessage(const char* aTxt)
  {
    char locBuff[N_PRINT_BUFF];
    char* ptr = locBuff;
    *ptr = 10; ptr++;
    strcpy(ptr, aTxt);
    int len = strlen(locBuff); locBuff[len] = '\n';
    WriteIO(locBuff, len + 1);
  }

  void SvMessage2(const char* aTxt, bool aOnOff)
  {
    char locBuff[N_PRINT_BUFF];
    char* ptr = locBuff;
    *ptr = 10; ptr++;
    strcpy(ptr, aTxt);
    if (aOnOff)
      strcat(ptr, " ON");
    else
      strcat(ptr, " OFF");
    int len = strlen(locBuff); locBuff[len] = '\n';
    WriteIO(locBuff, len + 1);
  }

  void WriteSvI16(int aId, int16_t aData)
  {
    uint8_t* ptr = buff + idx;
    *ptr = aId + 10; ptr++;
    *(int16_t*)ptr = aData;
    idx += 3;
  }

  void WriteSvF(int aId, float aData)
  {
    uint8_t* ptr = buff + idx;
    *ptr = aId + 20; ptr++;
    *(float*)ptr = aData;
    idx += 5;
  }

  uint8_t ReadByte()
  {
#ifdef USE_UDP
    return udp.ReadByte();
#else
    uint8_t val;
    ReadIO(&val, 1);
    return val;
#endif
  }

  int16_t ReadI16()
  {
#ifdef USE_UDP
    return udp.ReadI16();
#else
    int16_t val;
    ReadIO(&val, 2);
    return val;
#endif
  }

  void ReadI16Ary(int16_t* ary, int len)
  {
#ifdef USE_UDP
    return udp.ReadI16Ary(ary,len);
#else
    ReadIO(ary, 2 * len);
#endif
  }

  SvProtocol3& rdI16(int& val)
  {
    val = ReadI16();
    return *this;
  }
  
  float ReadI16SC(float aFactor)
  {
    return aFactor * (float)ReadI16();
  }

  float ReadF()
  {
#ifdef USE_UDP
    return udp.ReadF();
#else
    float val;
    ReadIO(&val, 4);
    return val;
#endif
  }

  SvProtocol3& rdF(float& val)
  {
    val = ReadF();
    return *this;
  }

  SvProtocol3& WrB(uint8_t aVal)
  {
    uint8_t* ptr = buff + idx;
    *ptr = aVal; idx++;
    return *this;
  }

  SvProtocol3& WrF(float aVal)
  {
    uint8_t* ptr = buff + idx;
    *(float*)ptr = aVal; idx += 4;
    return *this;
  }

  SvProtocol3& WrStr(const char* aStr)
  {
    int len = strlen(aStr);
    uint8_t* ptr = buff + idx;
    strcpy((char*)ptr, aStr);
    *(ptr + len) = '\n';
    idx += len + 1;
    return *this;
  }
};

extern "C" {
int wifi_init_sta();
void wifi_init_softap(char* aName, int aChan);
void print_own_ip_addr();
int InitUdp();
}

inline void InitStation()
{
#ifdef USE_UDP
  // InitNVS(); done in InitRtEnvHL()
  if (-1 == wifi_init_sta()) MyError("wifi");
  svSock = InitUdp();
  if (svSock == -1) MyError("sock");
#endif
}

inline void InitSoftAp(const char* aName, int aChan)
{
#ifdef USE_UDP
  // InitNVS(); done in InitRtEnvHL()
  wifi_init_softap((char*)aName, aChan);
  print_own_ip_addr();
  svSock = InitUdp();
  if (svSock == -1) MyError("sock");
#endif
}

extern "C" inline void UdpConnCB(int val)
{

}

// SV-Id Ranges and DataTypes for SvVis3 Visualisation-Tool
//----------------------------------------------------------
// Id = 10       : string
// Id = 1 .. 9   : format 3.13  2 Bytes
// Id = 11 .. 20 : short        2 Bytes
// Id = 21 .. 30 : float        4 Bytes

// Reserved Commands
// 50 - 59: reserved
// 50: reset uC
// 52: disconnect WiFi
// 55: JoyStick X Y
// 200: PID-Parameters
