#include "fault_trace.hpp"

#if ENABLE_FAULT_DIAGNOSTICS
#include <Arduino.h>
#include <CrashReport.h>

extern "C" void unused_interrupt_vector();
extern unsigned long _estack;
extern unsigned long _stext, _etext, _stextload;

namespace {
constexpr uint32_t TRACE_MAGIC = 0x4652544Fu; // FRTO: FreeRTOS-aware fault frame
constexpr uint32_t STACKING_ERRORS = (1u << 3) | (1u << 4) | (1u << 5) |
                                    (1u << 11) | (1u << 12) | (1u << 13);
constexpr uint32_t REGISTERS_MAGIC = 0x46524734u;
constexpr unsigned CODE_WORDS = 16;
constexpr unsigned CODE_LIVE = 12;
constexpr unsigned CODE_FLASH = CODE_LIVE + CODE_WORDS;
constexpr unsigned REGISTER_WORDS = CODE_FLASH + CODE_WORDS;
// DMAMEM is NOLOAD in the Teensy linker script: retain this across soft reset.
// Read it only with both a matching marker and checksum.
struct FaultRegisters {
  // R0..R3/R12/xPSR, PC, CFSR/MMFAR/BFAR,
  // code base/valid, then live ITCM and corresponding flash words.
  uint32_t value[REGISTER_WORDS];
  uint32_t checksum;
  uint32_t magic;
};
DMAMEM static volatile FaultRegisters fault_registers;
static bool itcm_write_guard = false;
static unsigned itcm_guard_status = 0; // 0=disabled, 1=armed, 3=layout

// Run from flash while changing the permissions of executable RAM. Only accept
// the existing Teensy core's region 1 layout, preserving all other attributes.
FLASHMEM __attribute__((noinline)) void protect_itcm() {
  if(!ENABLE_ITCM_WRITE_GUARD) return;
  // Do not infer debugger attachment from DHCSR.C_DEBUGEN: Teensy can leave
  // it set without a debugger. Code patching requires opting out explicitly.
  itcm_guard_status = 3;
  uint32_t primask;
  asm volatile("mrs %0, primask\ncpsid i" : "=r"(primask) :: "memory");
  const uint32_t region = SCB_MPU_RNR;
  const uint32_t control = SCB_MPU_CTRL;
  SCB_MPU_RNR = 1;
  const uint32_t attributes = SCB_MPU_RASR;
  const uint32_t access = attributes & SCB_MPU_RASR_AP(7);
  const uint32_t layout_mask = SCB_MPU_RASR_XN | SCB_MPU_RASR_SRD(255) |
      SCB_MPU_RASR_SIZE(31) | SCB_MPU_RASR_ENABLE;
  if((control & SCB_MPU_CTRL_ENABLE) &&
      (access == SCB_MPU_RASR_AP(3) || access == SCB_MPU_RASR_AP(6) || access == SCB_MPU_RASR_AP(7)) &&
      (SCB_MPU_RBAR & SCB_MPU_RBAR_ADDR_MASK) == 0 &&
      (attributes & layout_mask) == (SCB_MPU_RASR_SIZE(18) | SCB_MPU_RASR_ENABLE)) {
    asm volatile("dsb" ::: "memory");
    SCB_MPU_CTRL = control & ~SCB_MPU_CTRL_ENABLE;
    asm volatile("dsb\nisb" ::: "memory");
    SCB_MPU_RASR = (attributes & ~SCB_MPU_RASR_AP(7)) | SCB_MPU_RASR_AP(6);
    SCB_MPU_CTRL = control;
    asm volatile("dsb\nisb" ::: "memory");
    itcm_write_guard = (SCB_MPU_RASR & SCB_MPU_RASR_AP(7)) == SCB_MPU_RASR_AP(6);
    if(itcm_write_guard) itcm_guard_status = 1;
  }
  SCB_MPU_RNR = region;
  asm volatile("msr primask, %0" :: "r"(primask) : "memory");
}

uint32_t registers_checksum() {
  uint32_t checksum = REGISTERS_MAGIC;
  for(unsigned i = 0; i < REGISTER_WORDS; ++i) {
    checksum = ((checksum << 5) | (checksum >> 27)) ^ fault_registers.value[i];
  }
  return checksum;
}

bool readable_frame(uintptr_t address) {
  // The core registers occupy the first eight words of both basic and FP
  // exception frames. Do not dereference an invalid/failed exception stack.
  if((address & 3u) || (SCB_CFSR & STACKING_ERRORS)) return false;
  const uintptr_t size = 8 * sizeof(uint32_t);
  return (address >= 0x20000000u && address <= (uintptr_t)&_estack - size) ||
         (address >= 0x20200000u && address <= 0x20280000u - size);
}
}

// Called on MSP from the naked wrapper; no RTOS, allocation or Serial calls.
extern "C" __attribute__((used, noinline))
void record_fault_frame(const uint32_t *frame, uint32_t exception_return) {
  uint32_t ipsr;
  asm volatile("mrs %0, ipsr" : "=r"(ipsr));
  const bool valid = readable_frame((uintptr_t)frame);
  fault_registers.magic = 0;
  for(unsigned i = 0; i < 5; ++i) fault_registers.value[i] = valid ? frame[i] : 0;
  fault_registers.value[5] = valid ? frame[7] : 0;
  fault_registers.value[6] = valid ? frame[6] : 0;
  fault_registers.value[7] = SCB_CFSR;
  fault_registers.value[8] = SCB_MMFAR;
  fault_registers.value[9] = SCB_BFAR;
  const uintptr_t pc = fault_registers.value[6];
  const uintptr_t base = pc & ~(uintptr_t)15u;
  const bool code_valid = valid && base >= 32u && base >= (uintptr_t)&_stext &&
      base <= (uintptr_t)&_etext &&
      CODE_WORDS * sizeof(uint32_t) <= (uintptr_t)&_etext - base;
  fault_registers.value[10] = base;
  fault_registers.value[11] = code_valid;
  for(unsigned i = 0; i < CODE_WORDS; ++i) {
    fault_registers.value[CODE_LIVE + i] = code_valid ?
        reinterpret_cast<const volatile uint32_t *>(base)[i] : 0;
    fault_registers.value[CODE_FLASH + i] = code_valid ?
        reinterpret_cast<const volatile uint32_t *>((uintptr_t)&_stextload + base - (uintptr_t)&_stext)[i] : 0;
  }
  fault_registers.checksum = registers_checksum();
  fault_registers.magic = REGISTERS_MAGIC;
  arm_dcache_flush((void *)&fault_registers, sizeof(fault_registers));
  CrashReport.breadcrumb(2, valid ? frame[6] : 0); // Faulting PC
  CrashReport.breadcrumb(3, valid ? frame[5] : 0); // Stacked LR
  CrashReport.breadcrumb(4, exception_return);
  CrashReport.breadcrumb(5, (uint32_t)(uintptr_t)frame);
  CrashReport.breadcrumb(6, ipsr);
  CrashReport.breadcrumb(1, TRACE_MAGIC); // Commit only after all fields are saved.
}

extern "C" __attribute__((naked, used)) void freertos_fault_trace() {
  asm volatile(
      "cpsid i\n"
      "tst lr, #4\n"
      "ite eq\n"
      "mrseq r0, msp\n"
      "mrsne r0, psp\n"
      "mov r1, lr\n"
      "push {r4, lr}\n" // Keep the handler's MSP and EXC_RETURN intact.
      "bl record_fault_frame\n"
      "pop {r4, lr}\n"
      "b unused_interrupt_vector\n"); // Retain core fault reporting/auto reboot.
}

namespace DEBUG {
void install_fault_trace() {
  // Do not replace a debugger's or another component's custom fault handler.
  for(unsigned vector = 3; vector <= 6; ++vector) {
    if(_VectorsRam[vector] == unused_interrupt_vector) {
      _VectorsRam[vector] = freertos_fault_trace;
    }
  }
  protect_itcm();
  asm volatile("dsb\nisb" ::: "memory");
}

void print_fault_trace() {
  const volatile auto *trace =
      reinterpret_cast<const volatile crashreport_breadcrumbs_struct *>(0x2027FFC0u);
  if((trace->bitmask & 0x3fu) != 0x3fu || trace->value[0] != TRACE_MAGIC) return;
  Serial.printf("[FAULT] stacked PC=0x%08lx LR=0x%08lx\r\n",
      (unsigned long)trace->value[1], (unsigned long)trace->value[2]);
  Serial.printf("[FAULT] EXC_RETURN=0x%08lx SP=0x%08lx IPSR=%lu\r\n",
      (unsigned long)trace->value[3], (unsigned long)trace->value[4],
      (unsigned long)trace->value[5]);
  Serial.println("[FAULT] Use stacked PC; core CrashReport may use the wrong RTOS stack");
  if(fault_registers.magic == REGISTERS_MAGIC &&
      fault_registers.checksum == registers_checksum() &&
      fault_registers.value[6] == trace->value[1]) {
    Serial.printf("[FAULT] R0=0x%08lx R1=0x%08lx R2=0x%08lx\r\n",
        (unsigned long)fault_registers.value[0], (unsigned long)fault_registers.value[1],
        (unsigned long)fault_registers.value[2]);
    Serial.printf("[FAULT] R3=0x%08lx R12=0x%08lx xPSR=0x%08lx\r\n",
        (unsigned long)fault_registers.value[3],
        (unsigned long)fault_registers.value[4], (unsigned long)fault_registers.value[5]);
    Serial.printf("[FAULT] CFSR=0x%08lx MMFAR=0x%08lx BFAR=0x%08lx\r\n",
        (unsigned long)fault_registers.value[7], (unsigned long)fault_registers.value[8],
        (unsigned long)fault_registers.value[9]);
    if(fault_registers.value[11]) {
      unsigned differences = 0;
      for(unsigned i = 0; i < CODE_WORDS; ++i) {
        if(fault_registers.value[CODE_LIVE + i] != fault_registers.value[CODE_FLASH + i]) ++differences;
      }
      Serial.printf("[FAULT] ITCM snapshot base=0x%08lx differing_words=%u\r\n",
          (unsigned long)fault_registers.value[10], differences);
      for(unsigned i = 0; i < CODE_WORDS; i += 2) {
        Serial.printf("[CODE] %08lx RAM %08lx %08lx FLASH %08lx %08lx\r\n",
            (unsigned long)(fault_registers.value[10] + i * 4),
            (unsigned long)fault_registers.value[CODE_LIVE + i],
            (unsigned long)fault_registers.value[CODE_LIVE + i+1],
            (unsigned long)fault_registers.value[CODE_FLASH + i],
            (unsigned long)fault_registers.value[CODE_FLASH + i+1]);
      }
    }
  }
}

void print_fault_diagnostic_status() {
  Serial.printf("[ITCM] write_guard=%u status=%u (0=off 1=armed 3=layout)\r\n",
      (unsigned)itcm_write_guard, itcm_guard_status);
}
}

#endif // ENABLE_FAULT_DIAGNOSTICS
