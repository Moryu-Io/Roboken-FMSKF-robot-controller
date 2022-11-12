// RTOS
#include <FreeRTOS_TEENSY4.h>

// C++標準
#include <algorithm>

// Arduinoライブラリ
#include <ADC.h>
#include <DMAChannel.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "FD_adc.hpp"
#include "FD_task_main.hpp"
#include "global_config.hpp"

namespace FDT {

// ローカルパラメータ定義
constexpr uint32_t U32_FD_TASK_CTRL_FREQ_HZ = 50;
constexpr uint16_t U16_ADC_BUF_SAMPLE       = 4; // 1つのCHにつき何サンプル保存しておくか
constexpr uint16_t U16_ADC_BUF_SIZE         = SENSOR_DIR::SENS_NUM * U16_ADC_BUF_SAMPLE;

const uint16_t U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::SENS_NUM] = {2300, 2000, 2000, 2000, 2000, 2000, 2000, 2000};
const uint16_t U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::SENS_NUM] = {1000, 800, 800, 800, 800, 800, 800, 800};

// Peripheral設定
constexpr uint8_t U8_SENSOR_PWR_PIN = 41;

// ADC/DMA 設定
const uint16_t __attribute__((aligned(32))) U16_DMA_ADC_CH_CFG[SENSOR_DIR::SENS_NUM] = {8, 12, 11, 6, 5, 15, 0, 7}; // A1, A2, A3, A4, A5, A6, A7, A0 (最後が配列最初に格納される)
DMAMEM static volatile uint16_t __attribute__((aligned(32))) u16_adc_buffer[U16_ADC_BUF_SIZE];
myADC_DMA        *p_myAdc;
myADC_DMA::Config myAdcCfg = {
    .p_u16_adc_buf      = (uint16_t *)u16_adc_buffer,
    .u16_adc_buf_len    = U16_ADC_BUF_SIZE,
    .p_u16_dma_adc_ch   = (uint16_t *)U16_DMA_ADC_CH_CFG,
    .u16_dma_adc_ch_len = SENSOR_DIR::SENS_NUM,
};

// バッファ
uint16_t         U16_ADC_BUF_AVE[SENSOR_DIR::SENS_NUM] = {};
Info_FloorDetect INFO_NOW_FLOOR                        = {};

static void judge_sensor();

/**
 * @brief 割り込み関数のwrapper
 * @note 静的関数でないと関数ポインタを渡せないため
 * @note 現在は使用していない
 *
 */
void adc_dma_isr_wrapper() {
  p_myAdc->dma_isr();
}

/**
 * @brief タスク起動前の準備用関数
 *
 */
void prepare_task() {
  // 赤外線センサー用ADC&DMA起動
  p_myAdc = new myADC_DMA(myAdcCfg);
  p_myAdc->init(adc_dma_isr_wrapper);

  // 赤外線センサーPowerOn
  pinMode(U8_SENSOR_PWR_PIN, OUTPUT);
  digitalWrite(U8_SENSOR_PWR_PIN, HIGH);
}

/**
 * @brief タスク処理
 *
 * @param params
 */
void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / U32_FD_TASK_CTRL_FREQ_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(FDT_MAIN);

    UTIL::toggle_LED2();
    UTIL::toggle_LED3();

    /* 壁センサAD値の平均化処理 → メイジアンフィルタにしたい*/
  #if 1
    for(int i = 0; i < SENSOR_DIR::SENS_NUM; i++) {
      U16_ADC_BUF_AVE[i] = 0;
      for(int j = 0; j < U16_ADC_BUF_SAMPLE; j++) {
        U16_ADC_BUF_AVE[i] += u16_adc_buffer[(int)SENSOR_DIR::SENS_NUM * j + i];
      }
      U16_ADC_BUF_AVE[i] = U16_ADC_BUF_AVE[i] / U16_ADC_BUF_SAMPLE;
    }
  #endif

    for(int i = 0; i < SENSOR_DIR::SENS_NUM; i++) {

    }

    /* 判定処理 */
    judge_sensor();

    /* デバッグ処理 */
    #if 1
    // Filter値
    DEBUG_PRINT_FDT("[FDT]%d,%d,%d,%d,%d,%d,%d,%d\n",
                    U16_ADC_BUF_AVE[0],
                    U16_ADC_BUF_AVE[1],
                    U16_ADC_BUF_AVE[2],
                    U16_ADC_BUF_AVE[3],
                    U16_ADC_BUF_AVE[4],
                    U16_ADC_BUF_AVE[5],
                    U16_ADC_BUF_AVE[6],
                    U16_ADC_BUF_AVE[7]);
    #endif
    
    #if 0
    // 距離値
    DEBUG_PRINT_FDT("[FDT]%d,%d,%d,%d,%d,%d,%d,%d\n",
                    (int)get_now_walldist(SENSOR_DIR::FORWARD),
                    (int)get_now_walldist(SENSOR_DIR::BACK),
                    (int)get_now_walldist(SENSOR_DIR::RIGHT),
                    (int)get_now_walldist(SENSOR_DIR::LEFT),
                    (int)get_now_walldist(SENSOR_DIR::RIGHT_FORWARD),
                    (int)get_now_walldist(SENSOR_DIR::LEFT_FORWARD),
                    (int)get_now_walldist(SENSOR_DIR::RIGHT_BACK),
                    (int)get_now_walldist(SENSOR_DIR::LEFT_BACK));
    #endif

    // DEBUG_PRINT_FDT("%d, %d, %d\n",u16_adc_buffer[0], U16_ADC_BUF_AVE[0], (int)get_now_walldist(SENSOR_DIR::FORWARD));
    
    DEBUG_PRINT_PRC_FINISH(FDT_MAIN);
  }
}

static void judge_sensor() {
  // 前
  if(U16_ADC_BUF_AVE[SENSOR_DIR::FORWARD] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::FORWARD]) {
    INFO_NOW_FLOOR.u8_forward = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::FORWARD] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::FORWARD]) {
    INFO_NOW_FLOOR.u8_forward = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_forward = NO_DETECTED;
  }

  // 後
  if(U16_ADC_BUF_AVE[SENSOR_DIR::BACK] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::BACK]) {
    INFO_NOW_FLOOR.u8_back = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::BACK] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::BACK]) {
    INFO_NOW_FLOOR.u8_back = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_back = NO_DETECTED;
  }

  // 右
  if(U16_ADC_BUF_AVE[SENSOR_DIR::RIGHT] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::RIGHT]) {
    INFO_NOW_FLOOR.u8_right = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::RIGHT] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::RIGHT]) {
    INFO_NOW_FLOOR.u8_right = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_right = NO_DETECTED;
  }

  // 左
  if(U16_ADC_BUF_AVE[SENSOR_DIR::LEFT] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::LEFT]) {
    INFO_NOW_FLOOR.u8_left = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::LEFT] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::LEFT]) {
    INFO_NOW_FLOOR.u8_left = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_left = NO_DETECTED;
  }

  // 右前
  if(U16_ADC_BUF_AVE[SENSOR_DIR::RIGHT_FORWARD] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::RIGHT_FORWARD]) {
    INFO_NOW_FLOOR.u8_rForward = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::RIGHT_FORWARD] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::RIGHT_FORWARD]) {
    INFO_NOW_FLOOR.u8_rForward = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_rForward = NO_DETECTED;
  }

  // 左前
  if(U16_ADC_BUF_AVE[SENSOR_DIR::LEFT_FORWARD] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::LEFT_FORWARD]) {
    INFO_NOW_FLOOR.u8_lForward = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::LEFT_FORWARD] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::LEFT_FORWARD]) {
    INFO_NOW_FLOOR.u8_lForward = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_lForward = NO_DETECTED;
  }

  // 右後
  if(U16_ADC_BUF_AVE[SENSOR_DIR::RIGHT_BACK] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::RIGHT_BACK]) {
    INFO_NOW_FLOOR.u8_rBack = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::RIGHT_BACK] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::RIGHT_BACK]) {
    INFO_NOW_FLOOR.u8_rBack = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_rBack = NO_DETECTED;
  }

  // 左後
  if(U16_ADC_BUF_AVE[SENSOR_DIR::LEFT_BACK] > U16_ADC_THRE2_WALL_LIST[SENSOR_DIR::LEFT_BACK]) {
    INFO_NOW_FLOOR.u8_lBack = WALL_DETECTED;
  } else if(U16_ADC_BUF_AVE[SENSOR_DIR::LEFT_BACK] > U16_ADC_THRE2_FLOR_LIST[SENSOR_DIR::LEFT_BACK]) {
    INFO_NOW_FLOOR.u8_lBack = FLOOR_DETECTED;
  } else {
    INFO_NOW_FLOOR.u8_lBack = NO_DETECTED;
  }
}

/**
 * @brief Get the now FDinfo object
 *
 * @param _info
 */
void get_now_FDinfo(Info_FloorDetect &_info) {
  _info = INFO_NOW_FLOOR;
}

/**
 * @brief 指定方向の距離情報を取得
 * 
 * @param _dir : 取得したい方向
 * @return float : 距離情報[mm]
 */
float get_now_walldist(SENSOR_DIR _dir){
  uint16_t u16_ad = U16_ADC_BUF_AVE[_dir];
  if(u16_ad < 50) u16_ad = 50;

  // Sensor方向に平行な距離
  float dist_sns = 273.0f * 4096.0f / ((float)u16_ad * 3.3f);  // 273mm : 1V 電圧に反比例

  return dist_sns*0.8660254f;  // 30度傾いているので床に平行だと√3 / 2
}


}; // namespace FDT
