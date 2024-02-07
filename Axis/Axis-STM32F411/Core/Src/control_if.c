//
// Created by Joris on 29/01/2024.
//

#include "../../shared/packet.h"
#include "control_if.h"
#include "main.h"
#include "dma_support.h"

#include <memory.h>

void MX_SPI5_SPI_Only_Init(void);

union {
  axis_info_packet packet;
  char buffer[sizeof(axis_info_packet) + 2]; // extra bytes to ensure it's
} spi5_info_buffer;

#define RECEIVE_BUFFERS 4
union {
  axis_packet packet;
  char buffer[AXIS_PACKET_MAX_PACKET_LENGTH];
} spi5_receive_buffer[RECEIVE_BUFFERS];
volatile size_t spi5_receive_amount[RECEIVE_BUFFERS];
volatile size_t spi5_active_receive_buffer;
size_t spi5_processed_receive_buffer;

void control_if_init(void) {
  HAL_NVIC_SetPriority(EXTI1_IRQn, 1, 0);
  __HAL_GPIO_EXTI_CLEAR_IT(SPI5_NSS_Pin);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);


  memset(&spi5_info_buffer, 0, sizeof(spi5_info_buffer));
  spi5_info_buffer.packet.header.type = AXIS_PACKET_TYPE_INFO;
  spi5_info_buffer.packet.header.payload_length = sizeof(spi5_info_buffer.packet.info_data);
  spi5_info_buffer.packet.info_data.width = LEDS_X;
  spi5_info_buffer.packet.info_data.height = LEDS_Y * 2;
  spi5_info_buffer.packet.info_data.current_program = AXIS_PROGRAM_HUE;
  LL_DMA_SetMemoryAddress(TXDMA(SPI5), (uint32_t)&spi5_info_buffer);
  LL_DMA_SetDataLength(TXDMA(SPI5), sizeof(spi5_info_buffer));
  LL_DMA_SetPeriphAddress(TXDMA(SPI5), LL_SPI_DMA_GetRegAddr(SPI5));

  spi5_active_receive_buffer = 0;
  spi5_processed_receive_buffer = 0;
  LL_DMA_SetMemoryAddress(RXDMA(SPI5), (uint32_t)&spi5_receive_buffer[spi5_active_receive_buffer]);
  LL_DMA_SetDataLength(RXDMA(SPI5), sizeof(spi5_receive_buffer[0]));
  LL_DMA_SetPeriphAddress(RXDMA(SPI5), LL_SPI_DMA_GetRegAddr(SPI5));

  LL_SPI_EnableDMAReq_RX(SPI5);
  LL_SPI_EnableDMAReq_TX(SPI5);

  LL_DMA_EnableStreamEx(TXDMA(SPI5));
  LL_DMA_EnableStreamEx(RXDMA(SPI5));
  LL_SPI_Enable(SPI5);
}

void control_if_set_program(uint16_t program) {
  spi5_info_buffer.packet.info_data.current_program = program;
}

// This enforces about 2 us per transaction...
void control_if_IRQHandler(void) {
  // Reset sent DMA
  LL_DMA_DisableStream(TXDMA(SPI5));
  // this line should not be needed, deactivating CS should happen after transfer. This and the Cortex-M interrupt
  // latency should give more than enough time to have the RX buffer cleared by the DMA unit. Even on slow APB bus,
  // there should be at very least 8 ~ 10 clock cycles.
  // Checking also gives a lockup when SPI5 received more than DMA can receive, so must check TC interrupt as well...
  // while (SPI5->SR & SPI_SR_RXNE) { /* wait for DMA to finish */ }
  uint32_t remaining = LL_DMA_GetDataLength(RXDMA(SPI5));
  LL_DMA_DisableStream(RXDMA(SPI5));

  // Reset SPI peripheral?
  //LL_APB2_GRP1_ForceReset(LL_APB2_GRP1_PERIPH_SPI5);
  //LL_APB2_GRP1_ReleaseReset(LL_APB2_GRP1_PERIPH_SPI5);
  LL_SPI_Disable(SPI5);

  // MX_SPI
  //MX_SPI5_SPI_Only_Init();
  LL_DMA_EnableStreamEx(TXDMA(SPI5));
  LL_SPI_Enable(SPI5);

  // Save received and move to next received buffer
  spi5_receive_amount[spi5_active_receive_buffer] = sizeof(spi5_receive_buffer[0]) - remaining;
  spi5_active_receive_buffer = (spi5_active_receive_buffer + 1) % RECEIVE_BUFFERS;
  LL_DMA_SetMemoryAddress(RXDMA(SPI5), (uint32_t)&spi5_receive_buffer[spi5_active_receive_buffer]);
  LL_DMA_EnableStreamEx(RXDMA(SPI5));
}

void control_if_process(void) {
  size_t current = spi5_processed_receive_buffer;
  if (current != spi5_active_receive_buffer) {
    size_t amount = spi5_receive_amount[current];
    if (amount >= 4 && (amount - 4) >= spi5_receive_buffer[current].packet.header.payload_length) {
      axis_packet* p = &spi5_receive_buffer[current].packet;
      // Potential valid packet
      switch(p->header.type) {
        case AXIS_PACKET_TYPE_SET_PROGRAM:
          if (p->header.payload_length >= sizeof(axis_set_program_data)) {
            Program_Set((axis_program_t)p->program_set_data.program);
          }
          break;
        default:
          // ignore
          break;
      }
    }

    spi5_processed_receive_buffer = (spi5_processed_receive_buffer + 1) % RECEIVE_BUFFERS;
  }
}