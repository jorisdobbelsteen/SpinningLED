//
// Created by Joris on 10/08/2023.
//

#include "leds_driver.h"
#include "main.h"

static uint32_t g_ledbuffer_tx[4][LEDS_Y + 3];
static uint32_t g_ledbuffer_blank_tx[LEDS_Y + 3];
uint8_t g_brightness = BRIGHTNESS_NODETECT;

static uint32_t PickRgb(uint8_t h) {
  const unsigned int ha = (unsigned int)h * 3;
  const unsigned R = 24;
  const unsigned G = 16;
  const unsigned B = 8;
  switch(ha >> 8) {
    case 0: return (0xFF << R);
    case 1: return (0xFF << G);
    case 2: return (0xFF << B);
    default: return 0;
  }
}

static uint32_t HueToRgb(uint8_t h) {
  const unsigned int ha = (unsigned int)h * 6;
  const unsigned int x = ha & 0xFF;
  const unsigned int nx = ~ha & 0xFF;
  const unsigned R = 24;
  const unsigned G = 16;
  const unsigned B = 8;
  switch(ha >> 8) {
    case 0: return (0xFF << R) | (x << G);
    case 1: return (0xFF << G) | (nx << R);
    case 2: return (0xFF << G) | (x << B);
    case 3: return (0xFF << B) | (nx << G);
    case 4: return (0xFF << B) | (x << R);
    case 5: return (0xFF << R) | (nx << B);
    default: return 0;
  }
}

void leds_driver_init(void) {
  // Initialize g_ledbuffer_tx
  for (size_t s = 0; s < 4; ++s) {
    g_ledbuffer_tx[s][0] = 0;
    g_ledbuffer_tx[s][LEDS_Y + 1] = 0x010101E0;
    g_ledbuffer_tx[s][LEDS_Y + 2] = 0;
  }
  leds_driver_update_hue(0);

  // Initialize g_ledbuffer_blank_tx
  g_ledbuffer_blank_tx[0] = 0;
  g_ledbuffer_blank_tx[LEDS_Y + 1] = 0x010101E0;
  g_ledbuffer_blank_tx[LEDS_Y + 2] = 0;
  for (size_t i = 0; i < LEDS_Y; ++i) {
    g_ledbuffer_blank_tx[i] = 0x000000E1; // pretty much turn off fully...
  }

  // Prepare DMA
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS1), LL_SPI_DMA_GetRegAddr(hLEDS1));
  LL_DMA_SetDataLength(TXDMA(hLEDS1), sizeof(g_ledbuffer_blank_tx));
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS3), LL_SPI_DMA_GetRegAddr(hLEDS3));
  LL_DMA_SetDataLength(TXDMA(hLEDS3), sizeof(g_ledbuffer_blank_tx));
#if FOUR_CHANNEL
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS2), LL_SPI_DMA_GetRegAddr(hLEDS2));
  LL_DMA_SetDataLength(TXDMA(hLEDS2), sizeof(g_ledbuffer_blank_tx));
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS4), LL_SPI_DMA_GetRegAddr(hLEDS4));
  LL_DMA_SetDataLength(TXDMA(hLEDS4), sizeof(g_ledbuffer_blank_tx));
#endif

  // Turn on SPI busses and enable the DMA Request Bit
  LL_SPI_EnableDMAReq_TX(hLEDS1);
  LL_SPI_Enable(hLEDS1);
  LL_SPI_EnableDMAReq_TX(hLEDS3);
  LL_SPI_Enable(hLEDS3);
#if FOUR_CHANNEL
  LL_SPI_EnableDMAReq_TX(hLEDS2);
  LL_SPI_Enable(hLEDS2);
  LL_SPI_EnableDMAReq_TX(hLEDS4);
  LL_SPI_Enable(hLEDS4);
#endif
}

void leds_driver_update_hue(uint16_t hue) {
  const uint32_t LEDBASE = 0x000000E0 | (g_brightness >> 3);

  for (size_t s = 0; s < 4; ++s) {
    uint16_t hx = hue + s * 0x4000;
    uint32_t* const ledbuffer = &(g_ledbuffer_tx[s][1]);

    for (size_t i = 0; i < LEDS_Y; ++i) {
      ledbuffer[i] =
              //PickRgb
              HueToRgb
              (hx / 256) | LEDBASE;
      hx += 48;
    }
  }
}

__STATIC_INLINE void LL_DMA_EnableStreamEx(DMA_TypeDef *DMAx, uint32_t Stream) {
  // Interrupt clear
  switch (Stream) {
    case LL_DMA_STREAM_0: DMAx->LIFCR = 0x3D << 0; break;
    case LL_DMA_STREAM_1: DMAx->LIFCR = 0x3D << 6; break;
    case LL_DMA_STREAM_2: DMAx->LIFCR = 0x3D << 16; break;
    case LL_DMA_STREAM_3: DMAx->LIFCR = 0x3D << 22; break;
    case LL_DMA_STREAM_4: DMAx->HIFCR = 0x3D << 0; break;
    case LL_DMA_STREAM_5: DMAx->HIFCR = 0x3D << 6; break;
    case LL_DMA_STREAM_6: DMAx->HIFCR = 0x3D << 16; break;
    case LL_DMA_STREAM_7: DMAx->HIFCR = 0x3D << 22; break;
  }
  // Stream enable
  LL_DMA_EnableStream(DMAx, Stream);
}

void leds_driver_set_brightness(uint8_t brightness) {
  g_brightness = brightness;
}

void leds_driver_transmit(void) {
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS1), (uint32_t)&g_ledbuffer_tx[0][0]);
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS3), (uint32_t)&g_ledbuffer_tx[2][0]);
#if FOUR_CHANNEL
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS2), (uint32_t)&g_ledbuffer_tx[1][0]);
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS4), (uint32_t)&g_ledbuffer_tx[3][0]);
#endif

  LL_DMA_EnableStreamEx(TXDMA(hLEDS1));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS3));
#if FOUR_CHANNEL
  LL_DMA_EnableStreamEx(TXDMA(hLEDS2));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS4));
#endif
}

void leds_driver_transmit_blank(void) {
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS1), (uint32_t)&g_ledbuffer_blank_tx[0]);
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS3), (uint32_t)&g_ledbuffer_blank_tx[0]);
#if FOUR_CHANNEL
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS2), (uint32_t)&g_ledbuffer_blank_tx[0]);
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS4), (uint32_t)&g_ledbuffer_blank_tx[0]);
#endif

  LL_DMA_EnableStreamEx(TXDMA(hLEDS1));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS3));
#if FOUR_CHANNEL
  LL_DMA_EnableStreamEx(TXDMA(hLEDS2));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS4));
#endif
}
