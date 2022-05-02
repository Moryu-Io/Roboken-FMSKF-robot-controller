#ifndef UTIL_CACHE_HPP_
#define UTIL_CACHE_HPP_

#include <Arduino.h>

namespace UTIL {

#define SCB_CCSIDR_ASSOCIATIVITY_Pos 3U                                        /*!< SCB CCSIDR: Associativity Position */
#define SCB_CCSIDR_ASSOCIATIVITY_Msk (0x3FFUL << SCB_CCSIDR_ASSOCIATIVITY_Pos) /*!< SCB CCSIDR: Associativity Mask */
#define SCB_CCSIDR_NUMSETS_Pos       13U                                       /*!< SCB CCSIDR: NumSets Position */
#define SCB_CCSIDR_NUMSETS_Msk       (0x7FFFUL << SCB_CCSIDR_NUMSETS_Pos)      /*!< SCB CCSIDR: NumSets Mask */
#define SCB_DCCISW_WAY_Pos           30U                                       /*!< SCB DCCISW: Way Position */
#define SCB_DCCISW_WAY_Msk           (3UL << SCB_DCCISW_WAY_Pos)
#define SCB_DCCISW_SET_Pos           5U                              /*!< SCB DCCISW: Set Position */
#define SCB_DCCISW_SET_Msk           (0x1FFUL << SCB_DCCISW_SET_Pos) /*!< SCB DCCISW: Set Mask */
/* Cache Size ID Register Macros */
#define CCSIDR_WAYS(x) (((x)&SCB_CCSIDR_ASSOCIATIVITY_Msk) >> SCB_CCSIDR_ASSOCIATIVITY_Pos)
#define CCSIDR_SETS(x) (((x)&SCB_CCSIDR_NUMSETS_Msk) >> SCB_CCSIDR_NUMSETS_Pos)


static inline void SCB_DisableDCache(void) {
  uint32_t ccsidr;
  uint32_t sets;
  uint32_t ways;

  SCB_ID_CSSELR = (0U << 1U) | 0U;

  asm("dsb");
  ccsidr = SCB_ID_CCSIDR;
  SCB_CCR &= ~(uint32_t)SCB_CCR_DC; /* disable D-Cache */
                                    // clean & invalidate D-Cache
  sets = (uint32_t)(CCSIDR_SETS(ccsidr));
  do {
    ways = (uint32_t)(CCSIDR_WAYS(ccsidr));
    do {
      SCB_CACHE_DCCISW = (((sets << SCB_DCCISW_SET_Pos) & SCB_DCCISW_SET_Msk) |
                          ((ways << SCB_DCCISW_WAY_Pos) & SCB_DCCISW_WAY_Msk));
      //  __schedule_barrier();
      asm("nop");
      asm("nop");
    } while(ways--);
  } while(sets--);
  asm("dsb");
  asm("isb");
};

}; // namespace UTIL

#endif