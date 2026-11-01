#ifndef DEBUG_FAULT_TRACE_HPP_
#define DEBUG_FAULT_TRACE_HPP_

// Enable only when investigating faults. Disabled builds install no handlers,
// reserve no retained RAM, and emit no boot/fault diagnostic logs.
#ifndef ENABLE_FAULT_DIAGNOSTICS
#define ENABLE_FAULT_DIAGNOSTICS 0
#endif

// Optional companion to fault diagnostics: trap CPU writes into executable RAM.
// Leave disabled when using software breakpoints or other runtime code patches.
#ifndef ENABLE_ITCM_WRITE_GUARD
#define ENABLE_ITCM_WRITE_GUARD 0
#endif

#if ENABLE_FAULT_DIAGNOSTICS
namespace DEBUG {
void install_fault_trace();
void print_fault_trace();
void print_fault_diagnostic_status();
}
#endif

#endif
