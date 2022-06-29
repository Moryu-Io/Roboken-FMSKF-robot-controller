#include "AD_mode_positioning_seq_debug_data.hpp"

namespace ADT {

const ADTModePositioningSeq::PosCmdSeq POS_CMD_SEQ_DEBUG_0 = {
    .u32_id         = 0,
    .u8_cmd_seq_len = 4,
    .cmd_seq        = {
               {
                   .u32_dt_ms      = 0,
                   .fl_tgt_pos_deg = {0, 90.0f, 0, 0, 0},
        },
               {
                   .u32_dt_ms      = 1000,
                   .fl_tgt_pos_deg = {20.0f, 110.0f, 20.0f, 20.0f, 20.0f},
        },
               {
                   .u32_dt_ms      = 2000,
                   .fl_tgt_pos_deg = {-20.0f, 70.0f, -20.0f, -20.0f, -20.0f},
        },
               {
                   .u32_dt_ms      = 3000,
                   .fl_tgt_pos_deg = {0, 90.0f, 0, 0, 0},
        },
    },
};

ADTModePositioningSeq::PosCmdSeq *get_poscmdseq_debug() {
  return (ADTModePositioningSeq::PosCmdSeq *)&POS_CMD_SEQ_DEBUG_0;
}

} // namespace ADT