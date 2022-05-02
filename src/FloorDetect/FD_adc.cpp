#include <ADC.h>
#include <DMAChannel.h>
#include <string.h>

#include "FD_adc.hpp"

namespace FDT {

void myADC_DMA::init(void (*isr)(void)) {
  // buffer reset
  memset(r_cfg.p_u16_adc_buf, 0xFF, sizeof(uint16_t) * r_cfg.u16_adc_buf_len);

  // ADC0 config
  adc0->setAveraging(16);                                    // set number of averages
  adc0->setResolution(12);                                   // set bits of resolution
  adc0->setConversionSpeed(ADC_CONVERSION_SPEED::MED_SPEED); // change the conversion speed
  adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::MED_SPEED);     // change the sampling speed
  // adc->adc0->setReference(ADC_REF_1V2);
  // adc0->continuousMode();
  adc0->enableDMA();
  // adc0->setSoftwareTrigger();

  // DMA config
  p_dma_adc_data->begin(true);                    // allocate the DMA channel
  p_dma_adc_data->TCD->SADDR    = &ADC0_START.R0; // where to read from
  p_dma_adc_data->TCD->SOFF     = 0;              // source increment each transfer
  p_dma_adc_data->TCD->ATTR     = 0x0101;
  p_dma_adc_data->TCD->NBYTES   = sizeof(uint16_t); // bytes per transfer
  p_dma_adc_data->TCD->SLAST    = 0;
  p_dma_adc_data->TCD->DADDR    = r_cfg.p_u16_adc_buf; // where to write to
  p_dma_adc_data->TCD->DOFF     = sizeof(uint16_t);
  p_dma_adc_data->TCD->DLASTSGA = -sizeof(uint16_t) * r_cfg.u16_adc_buf_len;
  p_dma_adc_data->TCD->BITER    = r_cfg.u16_adc_buf_len;
  p_dma_adc_data->TCD->CITER    = r_cfg.u16_adc_buf_len;
  p_dma_adc_data->triggerAtHardwareEvent(DMAMUX_SOURCE_ADC1);
  // p_dma_adc_data->disableOnCompletion(); // require restart in code
  // p_dma_adc_data->interruptAtCompletion();
  // p_dma_adc_data->attachInterrupt(isr,3);

  p_dma_adc_cfg->begin(true); // allocate the DMA channel
  p_dma_adc_cfg->TCD->SADDR    = r_cfg.p_u16_dma_adc_ch;
  p_dma_adc_cfg->TCD->SOFF     = sizeof(uint16_t); // source increment each transfer (n bytes)
  p_dma_adc_cfg->TCD->ATTR     = 0x0101;
  p_dma_adc_cfg->TCD->NBYTES   = sizeof(uint16_t);
  p_dma_adc_cfg->TCD->SLAST    = -sizeof(uint16_t) * r_cfg.u16_dma_adc_ch_len;
  p_dma_adc_cfg->TCD->DADDR    = &ADC0_START.HC0;
  p_dma_adc_cfg->TCD->DOFF     = 0;
  p_dma_adc_cfg->TCD->DLASTSGA = 0;
  p_dma_adc_cfg->TCD->BITER    = r_cfg.u16_dma_adc_ch_len;
  p_dma_adc_cfg->TCD->CITER    = r_cfg.u16_dma_adc_ch_len;
  p_dma_adc_cfg->triggerAtTransfersOf(*p_dma_adc_data);
  p_dma_adc_cfg->triggerAtCompletionOf(*p_dma_adc_data);

  p_dma_adc_data->enable();
  p_dma_adc_cfg->enable();
}

/**
 * @brief DMA転送完了割り込み
 * @note なぜかたまに再起動するようになるので有効化しない
 *
 */
void myADC_DMA::dma_isr(void) {
  p_dma_adc_data->clearInterrupt();
  // p_dma_adc_data->TCD->DADDR = r_cfg.p_u16_adc_buf;
  p_dma_adc_data->enable();

  // uint32_t now_us = micros();
  // u32_dbg_dma_dtime = now_us - u32_dbg_dma_pretime;
  // u32_dbg_dma_pretime = now_us;
}

} // namespace FDT
