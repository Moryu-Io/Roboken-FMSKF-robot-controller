#ifndef AD_MODE_POSITIONING_SEQ_HPP_
#define AD_MODE_POSITIONING_SEQ_HPP_

#include <deque>

#include "AD_mode_base.hpp"
#include "global_config.hpp"

namespace ADT {

#define CMD_SEQ_BUF_LEN (4)

class ADTModePositioningSeq : public ADTModeBase {
public:
  struct PosCmd {
    uint32_t u32_dt_ms;
    float    fl_tgt_pos_deg[5];
  };

  struct PosCmdSeq {
    uint32_t u32_id;
    uint8_t  u8_cmd_seq_len;
    PosCmd   cmd_seq[16];
  };

public:
  ADTModePositioningSeq(){};

  void doInit() override;
  void update() override;
  void end() override{};

  void push_cmdseq(PosCmdSeq &_cmd);

  int32_t get_q_cmdseq_status(uint32_t _id);
  enum CmdStatus {
    PROCESSING = 0,
    DONE       = 1,
    NO_DATA    = 99,
  };

  enum State {
    STANDBY,
    MOVE_START,
    MOVING,
    COMPLETED
  };

private:
  /* 状態ごとの処理関数 */
  void exec_standby();
  void exec_move_start();
  void exec_moving();

  State nowState;

  PosCmdSeq cmd_seq_[CMD_SEQ_BUF_LEN];
  uint16_t  u16_seq_exec_idx_;   // 実行されているSeqのIDX
  uint16_t  u16_seq_write_head_; // 最後に書き込まれたSeq位置

  uint8_t  u8_nowcmd_idx_;  // Seqの中の実行中のコマンドIDX
  float    fl_move_deg_[5]; // 今回のコマンドでの移動量
  uint32_t u32_move_cnt_;   // 今回のコマンドでの移動時間[cycle数]

  uint32_t u32_total_move_ms_; // 現Seqで動いている時間[ms]

  PosCmd   now_cmd_;
  uint32_t u32_cycle_counter_;
};

}; // namespace ADT

#endif