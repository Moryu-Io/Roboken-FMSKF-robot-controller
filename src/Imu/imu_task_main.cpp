// RTOS
#include <FreeRTOS_TEENSY4.h>
#include <message_buffer.h>

// Arduinoライブラリ
#include <math.h>

// ローカル
#include "../Utility/util_mymath.hpp"
#include "global_config.hpp"
#include "imu_task_main.hpp"
#include "imu_if_wt901c.hpp"

namespace IMT {

// ローカルパラメータ定義
constexpr uint32_t U32_IM_TASK_CTRL_FREQ_HZ = 100;

// Peripheral設定


// デバイス設定

// IMU
static IMU_IF_WT901C imu_if;
// static IMU1 imu1;

// RTOS メッセージ
MessageBufferHandle_t p_MsgBufReq;
MSG_REQ               msgReq;

void prepare_task() {
  p_MsgBufReq = xMessageBufferCreate(IMT_MSG_REQ_BUFFER_SIZE * sizeof(MSG_REQ));
}

void main(void *params) {
  uint32_t debug_counter = 0;
  uint32_t loop_tick     = (int)configTICK_RATE_HZ / U32_IM_TASK_CTRL_FREQ_HZ;

  imu_if.init();

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(IMT_MAIN);

    imu_if.update();

    IMU_IF::Data _d = {};
    if(!imu_if.isError()){
      imu_if.getDataLatest(_d);
      DEBUG_PRINT_IMT("Acc:%d,%d,%d\n", (int)(_d.accel[0]*100.0f), (int)(_d.accel[1]*100.0f), (int)(_d.accel[2]*100.0f));
      DEBUG_PRINT_IMT("Gyr:%d,%d,%d\n", (int)_d.gyro[0], (int)_d.gyro[1], (int)_d.gyro[2]);
      DEBUG_PRINT_IMT("Ang:%d,%d,%d\n", (int)_d.angle[0], (int)_d.angle[1], (int)_d.angle[2]);
      DEBUG_PRINT_IMT("Qut:%d,%d,%d,%d\n", static_cast<int>(_d.qut[0]*100.0f),
                                           static_cast<int>(_d.qut[1]*100.0f),
                                           static_cast<int>(_d.qut[2]*100.0f),
                                           static_cast<int>(_d.qut[3]*100.0f));
    }else{
      DEBUG_PRINT_STR_IMT("No IMU data\n");
    }

    /* Msg処理 */
    if(xMessageBufferReceive(p_MsgBufReq, (void *)&msgReq, sizeof(MSG_REQ), 0) == sizeof(MSG_REQ)) {
      switch(msgReq.common.MsgId) {
      case REQ_MOVE_CONT_DIR: {
      } break; // REQ_MOVE_CONT_DIR
      default:
        break;
      }
    }

    /* 以下、デバッグ用 */
    if(debug_counter == 2) {

      debug_counter = 0;
    } else {
      debug_counter++;
    }

    DEBUG_PRINT_PRC_FINISH(IMT_MAIN);
  }
}


void get_status_now_imu(imu_data &imu_d){
  IMU_IF::Data _d = {};
  imu_if.getDataLatest(_d);

  imu_d.is_error = imu_if.isError();
  for(int i=0;i<3;i++){
    imu_d.acc[i] = _d.accel[i];
    imu_d.gyr[i] = _d.gyro[i];
    imu_d.mag[i] = _d.mag[i];
    imu_d.ang[i] = _d.angle[i];
  }
  for(int i=0;i<4;i++){
    imu_d.qut[i] = _d.qut[i];
  }
}

float get_status_now_yaw(){
  return imu_if.getYawDate();
}

void send_req_msg(MSG_REQ *_msg) {
  xMessageBufferSend(p_MsgBufReq, (void *)_msg, sizeof(MSG_REQ), 0);
}
}; // namespace IMT