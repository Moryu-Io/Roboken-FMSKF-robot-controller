#ifndef UTIL_GPTIMER_HPP_
#define UTIL_GPTIMER_HPP_

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

void     init_gptimer();
void     start_gptimer_cnt();
void     stop_gptimer_cnt();
uint32_t get_gptimer_cnt();

/* タスク処理時間計測用 */
void            init_debug_timer();
void            start_debug_cnt();
void            stop_debug_cnt();
inline uint32_t get_debug_cnt() { return GPT2_CNT; }    // 8MHzカウンタ

#ifdef __cplusplus
}
#endif

#endif