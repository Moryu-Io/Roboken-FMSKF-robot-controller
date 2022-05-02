#ifndef UTIL_LED_HPP_
#define UTIL_LED_HPP_

#include "global_config.hpp"

namespace UTIL {

inline void clear_bits(volatile uint32_t &reg, uint32_t bits) { reg &= ~bits; };
inline void set_bits(volatile uint32_t &reg, uint32_t bits) { reg |= bits; };
inline bool get_bits(volatile uint32_t &reg, uint32_t bits) { return (reg&bits) == bits; };

}; // namespace UTIL

#endif