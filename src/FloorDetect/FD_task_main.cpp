// RTOS
#include <FreeRTOS_TEENSY4.h>

// Arduinoライブラリ
#include <ADC.h>
#include <DMAChannel.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "FD_adc.hpp"
#include "FD_task_main.hpp"
#include "global_config.hpp"

namespace FDT {

#define ADC_BUF_SIZE 32

// Peripheral設定
constexpr uint8_t U8_SENSOR_PWR_PIN = 41;

// ADC/DMA 設定
const uint16_t __attribute__((aligned(32))) U16_DMA_ADC_CH_CFG[8] = {8, 12, 11, 6, 5, 15, 0, 7}; // A1, A2, A3, A4, A5, A6, A7, A0 (最後が配列最初に格納される)
DMAMEM static volatile uint16_t __attribute__((aligned(32))) u16_adc_buffer[ADC_BUF_SIZE];
myADC_DMA        *p_myAdc;
myADC_DMA::Config myAdcCfg = {
    .p_u16_adc_buf      = (uint16_t *)u16_adc_buffer,
    .u16_adc_buf_len    = ADC_BUF_SIZE,
    .p_u16_dma_adc_ch   = (uint16_t *)U16_DMA_ADC_CH_CFG,
    .u16_dma_adc_ch_len = 8,
};

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

  while(1) {
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);

    UTIL::toggle_LED2();
    UTIL::toggle_LED3();

    DEBUG_PRINT_FDT("[FDT]%d,%d,%d,%d,%d,%d,%d,%d,%d\n", micros(),
                    u16_adc_buffer[0],
                    u16_adc_buffer[1],
                    u16_adc_buffer[2],
                    u16_adc_buffer[3],
                    u16_adc_buffer[4],
                    u16_adc_buffer[5],
                    u16_adc_buffer[6],
                    u16_adc_buffer[7]);
  }
}

}; // namespace FDT
