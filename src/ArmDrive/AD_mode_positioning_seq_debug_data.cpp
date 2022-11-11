#include "AD_mode_positioning_seq_debug_data.hpp"

namespace ADT {

const ADTModePositioningSeq::PosCmdSeq POS_CMD_SEQ_DEBUG_0 = {
    .u32_id         = 0,
    .u8_cmd_seq_len = 4,
    .cmd_seq        = {
               {
                   .u32_dt_ms      = 0,
                   .fl_tgt_pos_deg = {0, 120.0f, -60.0f, 0, -10.0f},
        },
               {
                   .u32_dt_ms      = 100,
                   .fl_tgt_pos_deg = {20.0f, 45.0f, -45.0f, 45.0f, -45.0f},
        },
               {
                   .u32_dt_ms      = 1000,
                   .fl_tgt_pos_deg = {-20.0f, 90.0f, 0.0f, 0.0f, -45.0f},
        },
               {
                   .u32_dt_ms      = 1100,
                   .fl_tgt_pos_deg = {0, 120.0f, -60.0f, 0, -10.0f},
        },
    },
};

const ADTModePositioningSeq::PosCmdSeq POS_CMD_SEQ_DEBUG_1 = {
    .u32_id         = 0,
    .u8_cmd_seq_len = 2,
    .cmd_seq        = {
               {
                   .u32_dt_ms      = 0,
                   .fl_tgt_pos_deg = {0, 120.0f, -60.0f, 0, -10.0f},
        },
               {
                   .u32_dt_ms      = 1000,
                   .fl_tgt_pos_deg = {0, 90.0f, -60.0f, 0, -10.0f},
        },
    },
};

const ADTModePositioningSeq::PosCmdSeq POS_CMD_SEQ_DEBUG_2 = {
    .u32_id         = 0,
    .u8_cmd_seq_len = 4,
    .cmd_seq        = {
               {
                   .u32_dt_ms      = 0,
                   .fl_tgt_pos_deg = {0, 120.0f, -90.0f, 0, 45.0f},
        },
               {
                   .u32_dt_ms      = 1000,
                   .fl_tgt_pos_deg = {20.0f, 60.0f, -30.0f, 45.0f, -60.0f},
        },
               {
                   .u32_dt_ms      = 2000,
                   .fl_tgt_pos_deg = {-20.0f, 90.0f, 0.0f, 0.0f, 0.0f},
        },
               {
                   .u32_dt_ms      = 3000,
                   .fl_tgt_pos_deg = {0, 120.0f, -60.0f, 0, 45.0f},
        },
    },
};

ADTModePositioningSeq::PosCmdSeq *get_poscmdseq_debug() {
  // return (ADTModePositioningSeq::PosCmdSeq *)&POS_CMD_SEQ_DEBUG_0;
  return (ADTModePositioningSeq::PosCmdSeq *)&POS_CMD_SEQ_DEBUG_2;
}



} // namespace ADT