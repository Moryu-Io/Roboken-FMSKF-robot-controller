#ifndef FD_ADC_HPP_
#define FD_ADC_HPP_

#include <ADC.h>
#include <DMAChannel.h>

namespace FDT {

class myADC_DMA : private ADC {
public:
  struct Config {
    uint16_t *p_u16_adc_buf;
    uint16_t  u16_adc_buf_len;
    uint16_t *p_u16_dma_adc_ch;
    uint16_t  u16_dma_adc_ch_len;
  };

  myADC_DMA(Config &_cfg) : ADC(), r_cfg(_cfg) {
    p_dma_adc_data = new DMAChannel(false);
    p_dma_adc_cfg  = new DMAChannel(false);
  };

  void init(void (*isr)(void));
  void dma_isr();

  uint32_t get_dma_cplt_time_us() { return u32_dbg_dma_pretime; };
  uint32_t get_dma_deltatime_us() { return u32_dbg_dma_dtime; };

protected:
  Config &r_cfg;

  DMAChannel *p_dma_adc_data;
  DMAChannel *p_dma_adc_cfg;

  uint32_t u32_dbg_dma_pretime = 0;
  uint32_t u32_dbg_dma_dtime   = 0;
};

}; // namespace FDT

#endif