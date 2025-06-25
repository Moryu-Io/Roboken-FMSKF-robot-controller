#include "imu_if_wt901c.hpp"
#include <wit_c_sdk.h>
#include "../Utility/util_mymath.hpp"

// RTOS
#include <FreeRTOS_TEENSY4.h>

namespace IMT {

#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define QUAT_UPDATE   0x10
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
      case q3:
        s_cDataUpdate |= QUAT_UPDATE;
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

  Data buf = {};
  getDataImmediately(buf);
  q_init[0] =  sReg[q0] / 32768.0f;
  q_init[1] =  sReg[q1] / 32768.0f;
  q_init[2] =  sReg[q2] / 32768.0f;
  q_init[3] =  sReg[q3] / 32768.0f;
  
}

void IMU_IF_WT901C::kickCom() {
  P_SERIAL->clear();
}

void IMU_IF_WT901C::update() {
  is_error = !isComComp();

  if(!is_error){
    updateData();
  }
}

void IMU_IF_WT901C::updateData() {
  volatile uint8_t u8_write_page = u8_d_buf_read_page ? 0 : 1;
  volatile Data buf = {};

  for(int i = 0; i < 3; i++) {
    buf.accel[i] = static_cast<float>(sReg[AX + i]) / 32768.0f * 16.0f;
    buf.gyro[i]  = static_cast<float>(sReg[GX + i]) / 32768.0f * 2000.0f;
    buf.mag[i]   = static_cast<float>(sReg[HX + i]);
    buf.angle[i] = static_cast<float>(sReg[Roll + i]) / 32768.0f * 180.0f;
  }
  // x,y,z,w
  buf.qut[0] =  sReg[q0] / 32768.0f;
  buf.qut[1] =  sReg[q1] / 32768.0f;
  buf.qut[2] =  sReg[q2] / 32768.0f;
  buf.qut[3] =  sReg[q3] / 32768.0f;

  d_buf[u8_write_page].accel[0] =  buf.accel[0];
  d_buf[u8_write_page].accel[1] = -buf.accel[1];
  d_buf[u8_write_page].accel[2] = -buf.accel[2];

  d_buf[u8_write_page].gyro[0] =  buf.gyro[0];
  d_buf[u8_write_page].gyro[1] = -buf.gyro[1];
  d_buf[u8_write_page].gyro[2] = -buf.gyro[2];

  d_buf[u8_write_page].mag[0] =  buf.mag[0];
  d_buf[u8_write_page].mag[1] = -buf.mag[1];
  d_buf[u8_write_page].mag[2] = -buf.mag[2];

  d_buf[u8_write_page].angle[0] = UTIL::mymath::normalize_deg_0to360(buf.angle[0])-180.0f;
  d_buf[u8_write_page].angle[1] = buf.angle[1];
  d_buf[u8_write_page].angle[2] = buf.angle[2];
  
  d_buf[u8_write_page].qut[2] = -( q_init[3]*buf.qut[0] + q_init[2]*buf.qut[1] - q_init[1]*buf.qut[2] - q_init[0]*buf.qut[3]);
  d_buf[u8_write_page].qut[1] =  (-q_init[2]*buf.qut[0] + q_init[3]*buf.qut[1] + q_init[0]*buf.qut[2] - q_init[1]*buf.qut[3]);
  d_buf[u8_write_page].qut[0] = -( q_init[1]*buf.qut[0] - q_init[0]*buf.qut[1] + q_init[3]*buf.qut[2] - q_init[2]*buf.qut[3]);
  d_buf[u8_write_page].qut[3] =  ( q_init[0]*buf.qut[0] + q_init[1]*buf.qut[1] + q_init[2]*buf.qut[2] + q_init[3]*buf.qut[3]);

  u8_d_buf_read_page = u8_write_page;
}


bool IMU_IF_WT901C::isComComp() {
  volatile uint32_t vu32_pre_cnt = get_gptimer_cnt();

  while(P_SERIAL->available() && !isTimeUp(vu32_pre_cnt)) {
    WitSerialDataIn(P_SERIAL->read());
  }
  if(s_cDataUpdate & QUAT_UPDATE) {
    s_cDataUpdate = 0;
    return true;
  }
  return false;
}

void IMU_IF_WT901C::getDataLatest(Data &_d) {
  _d = d_buf[u8_d_buf_read_page];
}

void IMU_IF_WT901C::getDataImmediately(Data &_d) {
  WitReadReg(q0, 4);

  while(!isComComp()){
    ;
  }

  updateData();
  getDataLatest(_d);
}

float IMU_IF_WT901C::getYawDate(){
  return d_buf[u8_d_buf_read_page].angle[2];
}

void IMU_IF_WT901C::setImuConfigReg(ConfigReg &_cfg) {
}

} // namespace IMT