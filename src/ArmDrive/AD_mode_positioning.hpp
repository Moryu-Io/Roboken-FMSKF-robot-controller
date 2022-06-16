#ifndef AD_MODE_POSITIONING_HPP_
#define AD_MODE_POSITIONING_HPP_

#include <deque>

#include "AD_mode_base.hpp"
#include "global_config.hpp"

namespace ADT {

class ADTModePositioning : public ADTModeBase {
public:
  struct PosCmd {
    uint32_t u32_id;
    uint32_t u32_dt_ms;
    float    fl_tgt_pos_deg[5];
  };

public:
  ADTModePositioning(){};

  void doInit() override;
  void update() override;
  void end() override{};

  void push_cmd(PosCmd &_cmd);

  int32_t get_q_cmd_status(uint32_t _id);
  enum CmdStatus {
    PROCESSING = 0x00,
    DONE       = 0x01,
    NO_DATA    = 0x99,
  };

  enum State {
    STANDBY,
    MOVING,
    COMPLETED
  };

private:
  /* 状態ごとの処理関数 */
  void exec_standby();
  void exec_moving();

  State nowState;

  std::deque<PosCmd> cmd_q_;
  PosCmd             now_cmd_;
  float              fl_move_deg_[5]; // 今回のコマンドでの移動量
  uint32_t           u32_move_cnt_;   // 今回のコマンドでの移動時間[cycle数]

  uint32_t u32_prev_cmd_id_[2]; // 過去コマンドID保存(暫定)

  uint32_t u32_cycle_counter_;

  const uint32_t CU32_Q_MAX_NUM = 4;
};

}; // namespace ADT

#endif