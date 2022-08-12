#ifndef UTIL_GPTIMER_HPP_
#define UTIL_GPTIMER_HPP_

#include <Arduino.h>

#ifdef __cplusplus
extern "C"{
#endif

void init_gptimer();
void start_gptimer_cnt();
void stop_gptimer_cnt();
uint32_t get_gptimer_cnt();

#ifdef __cplusplus
}
#endif

#endif