// RTOS
#include <FreeRTOS_TEENSY4.h>

// C++標準

// Arduinoライブラリ
#include <SdFat.h>

// ローカル
#include "../Utility/util_led.hpp"
#include "Logger_task_main.hpp"
#include "global_config.hpp"

namespace LGT {

// ローカルパラメータ定義
constexpr uint32_t U32_LOGGER_TASK_CTRL_FREQ_HZ = 50;
constexpr uint32_t U32_SD_WRITE_BUF_LEN         = 4096; // 4kbyteで2msぐらい書き込み時間
constexpr uint32_t U32_LOGFILE_MAX_NUM          = 100000;

// Peripheral設定
//constexpr int SD_CHIP_SELECT = BUILTIN_SDCARD;

// 変数
bool   IS_SD_EXIST     = false;
String STR_LOGFILENAME = "LOG.txt";
SdFs SD;

// バッファ
struct Sd_Write_Buf {
  char     ch_buf[U32_SD_WRITE_BUF_LEN]; // バッファ
  uint32_t u32_bufidx;                   // 現在書き込まれたIndex
};

Sd_Write_Buf  ST_SD_WRITE_BUF[2]       = {};
volatile bool IS_SD_WRITE_LOCKED       = false; // 切り替えの瞬間のみLockする
uint8_t       U8_SD_WRITE_BUF_NOW_PAGE = 0;     // 現在書き込んでいるページ

// ファイル内関数定義
static String generate_logfilename();
static bool print_log_to_sd(const char *logchar, size_t ch_num);

void prepare_task() {
  if(!SD.begin(SdioConfig(FIFO_SDIO))) {
    IS_SD_EXIST = false;
    Serial.println("Card failed, or not present");
  } else {
    /* SDある場合 */
    IS_SD_EXIST = true;

    STR_LOGFILENAME = generate_logfilename();

#if 0
    for(int i=0;i<U32_SD_WRITE_BUF_LEN;i++){
      ST_SD_WRITE_BUF[0].ch_buf[i] = (i%10) + 0x30;
      ST_SD_WRITE_BUF[1].ch_buf[i] = (i%10) + 0x30;
    }
    ST_SD_WRITE_BUF[0].u32_bufidx = U32_SD_WRITE_BUF_LEN;
    ST_SD_WRITE_BUF[1].u32_bufidx = U32_SD_WRITE_BUF_LEN;
#endif

  }
}

void main(void *params) {
  uint32_t loop_tick = (int)configTICK_RATE_HZ / U32_LOGGER_TASK_CTRL_FREQ_HZ;

  auto xLastWakeTime = xTaskGetTickCount();
  while(1) {
    vTaskDelayUntil(&xLastWakeTime, loop_tick);
    DEBUG_PRINT_PRC_START(LOG_MAIN);

    if(!IS_SD_EXIST) {

    } else {
      /* SDある場合 */

      if(ST_SD_WRITE_BUF[U8_SD_WRITE_BUF_NOW_PAGE].u32_bufidx >= (U32_SD_WRITE_BUF_LEN>>1)){
        /* バッファが半分以上埋まっていたら書き込みBuffer面の切り替え */

        // 先に次にためるバッファ面を初期化して切り替え
        // 切り替えの瞬間だけバッファへの書き込みをLock
        IS_SD_WRITE_LOCKED                                   = true;
        uint8_t sdpush_page                                  = U8_SD_WRITE_BUF_NOW_PAGE;
        U8_SD_WRITE_BUF_NOW_PAGE                             = U8_SD_WRITE_BUF_NOW_PAGE ^ 1;
        ST_SD_WRITE_BUF[U8_SD_WRITE_BUF_NOW_PAGE].u32_bufidx = 0;
        IS_SD_WRITE_LOCKED                                   = false;

        // これまでためてきたバッファをSDカードに書き込み
        print_log_to_sd(ST_SD_WRITE_BUF[sdpush_page].ch_buf,
                        ST_SD_WRITE_BUF[sdpush_page].u32_bufidx);
      }

    }

    DEBUG_PRINT_PRC_FINISH(LOG_MAIN);
  }
}

static String generate_logfilename() {
  int    _filenum = 0;
  String _str_logfile_num;
  String _str_logfile_name;
  while(1) {
    _str_logfile_num = "0000";
    _str_logfile_num += String(_filenum);

    _str_logfile_name = "LOG";
    _str_logfile_name += _str_logfile_num.substring(_str_logfile_num.length() - 5);

    _str_logfile_name += ".txt";
    if(!SD.exists(_str_logfile_name.c_str())) {
      break;
    }

    _filenum++;
    if(_filenum >= U32_LOGFILE_MAX_NUM) {
      // LOGFILEがいっぱいの場合
      break;
    }
  }

  return _str_logfile_name;
}

static bool print_log_to_sd(const char *logchar, size_t ch_num) {
  if(ch_num == 0) return true;

  FsFile logFile = SD.open(STR_LOGFILENAME.c_str(), FILE_WRITE);
  if(logFile) {
    logFile.write(logchar, ch_num);
    logFile.close();

    return true;
  }

  logFile.close();
  return false;
}


void push_buffer(char *_buf, uint32_t _size) {
  if(IS_SD_WRITE_LOCKED) return;

  uint32_t _u32_head     = ST_SD_WRITE_BUF[U8_SD_WRITE_BUF_NOW_PAGE].u32_bufidx;
  uint32_t _u32_rest_len = U32_SD_WRITE_BUF_LEN - _u32_head;
  if(_u32_rest_len >= _size) {
    /* バッファ残りより少ない量の書き込みの場合 */
    memcpy(&ST_SD_WRITE_BUF[U8_SD_WRITE_BUF_NOW_PAGE].ch_buf[_u32_head], _buf, _size);
    ST_SD_WRITE_BUF[U8_SD_WRITE_BUF_NOW_PAGE].u32_bufidx += _size;
  } else {
    /* バッファ残りより多い場合、書き込める場所まで書き込む */
    memcpy(&ST_SD_WRITE_BUF[U8_SD_WRITE_BUF_NOW_PAGE].ch_buf[_u32_head], _buf, _u32_rest_len);
    ST_SD_WRITE_BUF[U8_SD_WRITE_BUF_NOW_PAGE].u32_bufidx += _u32_rest_len;
  }
}

} // namespace LGT