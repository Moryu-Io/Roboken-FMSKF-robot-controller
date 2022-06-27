#ifndef FD_TASK_MAIN_HPP_
#define FD_TASK_MAIN_HPP_

namespace FDT {

enum SENSOR_DIR {
  FORWARD,
  BACK,
  RIGHT,
  LEFT,
  RIGHT_FORWARD,
  LEFT_FORWARD,
  RIGHT_BACK,
  LEFT_BACK,
  SENS_NUM,
};

#define NO_DETECTED    (0) // 何も検知していない
#define FLOOR_DETECTED (1) // 床検知
#define WALL_DETECTED  (2) // 壁検知(相手か？)

struct Info_FloorDetect {
  uint8_t u8_rForward;
  uint8_t u8_lForward;
  uint8_t u8_rBack;
  uint8_t u8_lBack;
  uint8_t u8_right;
  uint8_t u8_left;
  uint8_t u8_forward;
  uint8_t u8_back;
};

// 外部タスク用の壁情報取得
void get_now_FDinfo(Info_FloorDetect &_info);
float get_now_walldist(SENSOR_DIR _dir);

void prepare_task();
void main(void *params);

}; // namespace FDT

#endif