// RTOS
#include <FreeRTOS_TEENSY4.h>

// Arduinoライブラリ
#include <TsyDMASPI.h>

// ローカル
#include "global_config.hpp"
#include "VD_imu_if_mpu6500.hpp"
#include "VD_task_main.hpp"

namespace VDT {

// Peripheral設定
constexpr uint8_t U8_IMU1_CS_PIN = 37;
constexpr uint8_t U8_IMU2_CS_PIN = 36;

// デバイス設定
class IMU1 : public IMU_IF_MPU6500 {
public:
  void init() override {
    TsyDMASPI0.begin(U8_IMU1_CS_PIN, SPISettings(1000000, MSBFIRST, SPI_MODE3), true);
    LSBtoG     = 1.0f/16834.0f;
    LSBtoRADPS = 3.141592f/(180.0f*131.0f);
  }

protected:
  void kickSpiDma(uint8_t size) {
    TsyDMASPI0.queue(u8_tx_buf, u8_rx_buf, (size_t)size, U8_IMU1_CS_PIN);
  };
  bool isCompSpiDmaQueue() {
    return TsyDMASPI0.remained() == 0;
  };
};


// インスタンス
static IMU1 imu1;

void prepare_task() {
    imu1.init();
}

void main(void *params) {
  IMU_IF::Data _imu_d = {};
  while(1) {
    vTaskDelay((1000L * configTICK_RATE_HZ) / 1000L);
    imu1.getDataImmediately(_imu_d);
    DEBUG_PRINT_VDT_IMU("%0.2f\n", _imu_d.accel_y);
  }
}

}; // namespace VDT