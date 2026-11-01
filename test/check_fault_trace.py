#!/usr/bin/env python3
"""Host-check retained fault snapshots and MPU setup with fake MMIO.

ARM instructions are stubbed; verify their emitted code separately in the ELF.
"""
from pathlib import Path
import re, subprocess
source=(Path(__file__).resolve().parents[1]/'src/Debug/fault_trace.cpp').read_text()
source=re.sub(r'^#include .*\n', '', source, flags=re.M)
source=re.sub(r'extern "C" __attribute__\(\(naked, used\)\) void freertos_fault_trace\(\) \{.*?\n\}', 'extern "C" void freertos_fault_trace() {}', source, flags=re.S)
source=source.replace('asm volatile("mrs %0, ipsr" : "=r"(ipsr));', 'ipsr = test_ipsr;')
source=source.replace('asm volatile("dsb\\nisb" ::: "memory");', '')
source=source.replace('asm volatile("dsb" ::: "memory");', '')
source=source.replace('asm volatile("mrs %0, primask\\ncpsid i" : "=r"(primask) :: "memory");', 'primask=test_primask; test_primask=1;')
source=source.replace('asm volatile("msr primask, %0" :: "r"(primask) : "memory");', 'test_primask=primask;')
prefix=r'''
#include <cassert>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <cstdarg>
#include <string>
#include <sys/mman.h>
#define DMAMEM
#define FLASHMEM
#define ENABLE_FAULT_DIAGNOSTICS 1
extern unsigned long _estack;
asm(".globl _estack\n.set _estack, 0x20038000");
volatile uint32_t SCB_CFSR=0x82;
asm(".globl _stext\n.set _stext, 0x10000");
asm(".globl _etext\n.set _etext, 0x20000");
asm(".globl _stextload\n.set _stextload, 0x60010000");
volatile uint32_t SCB_MMFAR=0, SCB_BFAR=0;
volatile uint32_t SCB_MPU_RNR=9, SCB_MPU_CTRL=1, SCB_MPU_RBAR=0, SCB_MPU_RASR=0;
#define SCB_MPU_CTRL_ENABLE 1u
#define SCB_MPU_RASR_ENABLE 1u
#define SCB_MPU_RASR_AP(n) ((uint32_t)(n)<<24)
#define SCB_MPU_RASR_XN (1u<<28)
#define SCB_MPU_RASR_SRD(n) ((uint32_t)(n)<<8)
#define SCB_MPU_RASR_SIZE(n) ((uint32_t)(n)<<1)
#define SCB_MPU_RBAR_ADDR_MASK 0xffffffe0u
uint32_t test_primask=0, test_ipsr=4;
void arm_dcache_flush(void*, size_t) {}
void (*_VectorsRam[256])();
struct crashreport_breadcrumbs_struct { uint32_t bitmask, value[6], checksum; };
struct {
 void breadcrumb(unsigned n, uint32_t v) {
  auto *p=reinterpret_cast<crashreport_breadcrumbs_struct *>(0x2027ffc0u);
  p->value[n-1]=v; p->bitmask|=1u<<(n-1);
 }
} CrashReport;
struct {
 std::string text;
 void printf(const char *format, ...) {
  char buf[1024]; va_list args; va_start(args, format); vsnprintf(buf,sizeof buf,format,args); va_end(args); text+=buf;
 }
 void println(const char *s) { text+=s; text+='\n'; }
} Serial;
extern "C" void unused_interrupt_vector() {}
void custom_fault_handler() {}
'''
suffix=r'''
int main() {
 for(auto address:{0x10000u,0x60010000u,0x20000000u,0x20200000u}) {
  const size_t size=(address==0x10000u || address==0x60010000u) ? 0x10000u : 0x80000u;
  assert(mmap(reinterpret_cast<void *>(uintptr_t(address)),size,PROT_READ|PROT_WRITE,
      MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0)==reinterpret_cast<void *>(uintptr_t(address)));
 }
 auto reset=[] {
  for(auto &vector:_VectorsRam) vector=unused_interrupt_vector;
  itcm_write_guard=false; itcm_guard_status=0;
  SCB_MPU_CTRL=5; SCB_MPU_RNR=9; SCB_MPU_RBAR=0;
  SCB_MPU_RASR=SCB_MPU_RASR_AP(3) | (1u<<19) | SCB_MPU_RASR_SIZE(18) | 1u;
  test_primask=0;
 };
 reset(); _VectorsRam[4]=custom_fault_handler; DEBUG::install_fault_trace();
 assert(_VectorsRam[3]==freertos_fault_trace && _VectorsRam[4]==custom_fault_handler);
 assert(_VectorsRam[5]==freertos_fault_trace && _VectorsRam[6]==freertos_fault_trace);
 assert(itcm_write_guard == bool(ENABLE_ITCM_WRITE_GUARD));
 assert(SCB_MPU_RASR==(SCB_MPU_RASR_AP(ENABLE_ITCM_WRITE_GUARD ? 6 : 3) | (1u<<19) | SCB_MPU_RASR_SIZE(18) | 1u));
 assert(SCB_MPU_CTRL==5 && SCB_MPU_RNR==9 && test_primask==0);
 reset(); test_primask=1; protect_itcm(); assert(test_primask==1);
 reset(); SCB_MPU_RBAR=0x20000000u; protect_itcm(); assert(!itcm_write_guard && SCB_MPU_RNR==9);
 reset(); SCB_MPU_RASR|=SCB_MPU_RASR_XN; protect_itcm(); assert(!itcm_write_guard);
 reset(); SCB_MPU_CTRL=0; protect_itcm(); assert(!itcm_write_guard && SCB_MPU_CTRL==0);
 reset(); DEBUG::install_fault_trace();
 auto *frame=reinterpret_cast<uint32_t *>(0x20010000u);
 frame[0]=0x20004830u; frame[1]=1; frame[2]=0; frame[3]=0x20004630u;
 frame[4]=123; frame[5]=0x1e01; frame[6]=0x18866; frame[7]=0x21000000u;
 auto *live=reinterpret_cast<uint32_t *>(0x18860u);
 auto *flash=reinterpret_cast<uint32_t *>(0x60018860u);
 for(unsigned i=0;i<CODE_WORDS;++i) live[i]=flash[i]=0x12345678u+i;
 live[1]^=1;
 record_fault_frame(frame,0xffffffedu);
 assert(fault_registers.magic==REGISTERS_MAGIC && fault_registers.checksum==registers_checksum());
 assert(fault_registers.value[0]==0x20004830u && fault_registers.value[6]==0x18866u);
 assert(fault_registers.value[11]==1 && fault_registers.value[10]==0x18860u);
 assert(fault_registers.value[CODE_LIVE+1] != fault_registers.value[CODE_FLASH+1]);
 DEBUG::print_fault_trace();
 assert(Serial.text.find("differing_words=1")!=std::string::npos);
 assert(Serial.text.find("R0=0x20004830")!=std::string::npos);
 Serial.text.clear(); fault_registers.value[0]^=1; DEBUG::print_fault_trace();
 assert(Serial.text.find("R0=")==std::string::npos);
 for(uint32_t bad_pc:{0u,0x1fff0u,0x20000u,0x20000000u,0xfffffff0u}) {
  frame[6]=bad_pc; record_fault_frame(frame,0xfffffffdu); assert(fault_registers.value[11]==0);
 }
 frame[6]=0x18866;
 record_fault_frame(nullptr,0xfffffffdu); assert(fault_registers.value[6]==0);
 SCB_CFSR=1u<<4; record_fault_frame(frame,0xfffffffdu); assert(fault_registers.value[6]==0);
 puts("PASS: retained register/code snapshots, checksum, invalid PC/stack guards, custom handlers and MPU/mask preservation");
}
'''

import tempfile
with tempfile.TemporaryDirectory(prefix='check-fault-trace-') as directory:
    cpp=Path(directory)/'check.cpp'
    binary=Path(directory)/'check'
    cpp.write_text(prefix+source+suffix)
    for guard in (0, 1):
        subprocess.run(['g++','-std=c++14','-fno-pie','-no-pie','-Wall','-Wextra','-Werror',
                        f'-DENABLE_ITCM_WRITE_GUARD={guard}',str(cpp),'-o',str(binary)],check=True)
        subprocess.run([str(binary)],check=True)
    # Exercise the real default preprocessor path: the module must be empty.
    obj=Path(directory)/'disabled.o'
    actual=Path(__file__).resolve().parents[1]/'src/Debug/fault_trace.cpp'
    subprocess.run(['g++','-std=c++14','-c',str(actual),'-o',str(obj)],check=True)
    assert not subprocess.check_output(['nm',str(obj)],text=True).strip()
    print('PASS: default build has no diagnostic code or retained RAM')
