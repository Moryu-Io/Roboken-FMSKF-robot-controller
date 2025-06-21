#include "imu_if_wt901c.hpp"
#include <wit_c_sdk.h>

// RTOS
#include <FreeRTOS_TEENSY4.h>

namespace IMT {

#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80
static volatile char   s_cDataUpdate = 0;
static HardwareSerial *P_SERIAL      = &Serial6;

static void SensorUartSend(uint8_t *p_data, uint32_t uiSize) {
  P_SERIAL->write(p_data, uiSize);
  P_SERIAL->flush();
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
  int i;
  for(i = 0; i < uiRegNum; i++) {
    switch(uiReg) {
    case AZ:
      s_cDataUpdate |= ACC_UPDATE;
      break;
    case GZ:
      s_cDataUpdate |= GYRO_UPDATE;
      break;
    case HZ:
      s_cDataUpdate |= MAG_UPDATE;
      break;
    case Yaw:
      s_cDataUpdate |= ANGLE_UPDATE;
      break;
    default:
      s_cDataUpdate |= READ_UPDATE;
      break;
    }
    uiReg++;
  }
}
static void Delayms(uint16_t ucMs) {
  vTaskDelay(ucMs);
}

bool IMU_IF_WT901C::isTimeUp(uint32_t u32_init_cnt) {
  uint32_t u32_now_cnt = get_gptimer_cnt();

  uint32_t u32_dlt_cnt =
      (u32_now_cnt >= u32_init_cnt) ? u32_now_cnt - u32_init_cnt
                                   : 0x100000000 - u32_init_cnt + u32_now_cnt;

  return u32_dlt_cnt > u32_count_timeup;
}

void IMU_IF_WT901C::init() {
  P_SERIAL->begin(115200);
  WitInit(WIT_PROTOCOL_NORMAL, 0x50);
  WitSerialWriteRegister(SensorUartSend);
  WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
}

void IMU_IF_WT901C::kickCom() {
  P_SERIAL->clear();
}

void IMU_IF_WT901C::update() {
  is_error = !isComComp();
}

bool IMU_IF_WT901C::isComComp() {
  volatile uint32_t vu32_pre_cnt = get_gptimer_cnt();

  while(P_SERIAL->available() && !isTimeUp(vu32_pre_cnt)) {
    WitSerialDataIn(P_SERIAL->read());
  }
  if(s_cDataUpdate & ANGLE_UPDATE) {
    s_cDataUpdate = 0;
    return true;
  }
  return false;
}

void IMU_IF_WT901C::getDataLatest(Data &_d) {
  for(int i = 0; i < 3; i++) {
    _d.accel[i] = sReg[AX + i] / 32768.0f * 16.0f;
    _d.gyro[i]  = sReg[GX + i] / 32768.0f * 2000.0f;
    _d.mag[i]   = sReg[HX + i];
    _d.angle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
  }
  for(int i = 0; i < 4; i++) {
    _d.qut[i]   = sReg[q0 + i] / 32768.0f;
  }
}

void IMU_IF_WT901C::getDataImmediately(Data &_d) {
  WitReadReg(AX, 12);

  while(!isComComp()) {
    ;
  }
  getDataLatest(_d);
}

void IMU_IF_WT901C::setImuConfigReg(ConfigReg &_cfg) {
}

} // namespace IMT