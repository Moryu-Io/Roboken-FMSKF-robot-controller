#include "util_gptimer.hpp"

#ifdef __cplusplus
extern "C"{
#endif

void init_gptimer() {
  // Enable timer
  CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON); // Enable clock to GPT1 module
  GPT1_CR = 0;                                  // Disable for configuration
  GPT1_PR = 24 - 1;                             // Prescale 24 MHz clock by 24 => 1 MHz
  GPT1_CR = GPT_CR_CLKSRC(1)                    /* 24 MHz peripheral clock as clock source */
            | GPT_CR_FRR;                        /* Free-Run, do not reset */
//            | GPT_CR_ENMOD;                     /* Cnt and Prc after Enable */
}

void start_gptimer_cnt() {
  GPT1_CR |= GPT_CR_EN; /* Enable timer */
}

void stop_gptimer_cnt() {
  GPT1_CR = GPT1_CR & (~GPT_CR_EN); /* Disable timer */
}


uint32_t get_gptimer_cnt() {
  return GPT1_CNT;
}


void init_debug_timer() {
  // Enable timer
  CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON); // Enable clock to GPT2 module
  GPT2_CR = 0;                                  // Disable for configuration
  GPT2_PR = 3 - 1;                             // Prescale 24 MHz clock by 24 => 8 MHz
  GPT2_CR = GPT_CR_CLKSRC(1)                    /* 24 MHz peripheral clock as clock source */
            | GPT_CR_FRR;                        /* Free-Run, do not reset */
//            | GPT_CR_ENMOD;                     /* Cnt and Prc after Enable */
}

void start_debug_cnt() {
  GPT2_CR |= GPT_CR_EN; /* Enable timer */
}

void stop_debug_cnt() {
  GPT2_CR = GPT2_CR & (~GPT_CR_EN); /* Disable timer */
}


#ifdef __cplusplus
}
#endif
