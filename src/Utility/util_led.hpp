#ifndef UTIL_LED_HPP_
#define UTIL_LED_HPP_

#include <Arduino.h>

#define LED0_PIN (13)   // OnBoard(SCKと共通なので初めのテスト用のみ使用)
#define LED1_PIN (33)   // Status1
#define LED2_PIN (34)   // Status2
#define LED3_PIN (35)   // Status3


namespace UTIL {

inline void init_LEDpin(){
  // pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
}

inline void set_LED0(bool on) { /* digitalWrite(LED0_PIN, (on) ? HIGH : LOW); */ }
inline void set_LED1(bool on) { digitalWrite(LED1_PIN, (on) ? HIGH : LOW); }
inline void set_LED2(bool on) { digitalWrite(LED2_PIN, (on) ? HIGH : LOW); }
inline void set_LED3(bool on) { digitalWrite(LED3_PIN, (on) ? HIGH : LOW); }

inline void toggle_LED0() { /* digitalWrite(LED0_PIN, !digitalRead(LED0_PIN)); */ }
inline void toggle_LED1() { digitalWrite(LED1_PIN, !digitalRead(LED1_PIN)); }
inline void toggle_LED2() { digitalWrite(LED2_PIN, !digitalRead(LED2_PIN)); }
inline void toggle_LED3() { digitalWrite(LED3_PIN, !digitalRead(LED3_PIN)); }

}; // namespace UTIL

#endif