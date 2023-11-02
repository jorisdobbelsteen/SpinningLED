//
// Created by Joris on 10/08/2023.
//

/* Led Arm configuration (top view)
 *
 * === TWO ARMS ===============================================
 *
 * FOUR_CHANNEL = 0
 *                 <-
 *  channel LEDS1      ----OO----     channel LEDS3
 *  0 deg              ----OO----     180 deg interlaced
 *                                 ->
 *
 * === FOUR ARMS ==============================================
 *
 * FOUR_CHANNEL = 1
 *
 *                     channel LEDS2
 *                     90 deg interlaced
 *                 <-      ||
 *  channel LEDS1      ----OO----     channel LEDS3
 *  0 deg              ----OO----     180 deg
 *                         ||      ->
 *                     channel LEDS4
 *                     270 deg interlaced
 */

#include "leds_driver.h"
#include "main.h"

volatile size_t g_ledbuffer_tx_sent = 0;
static led_data_buffer_t g_ledbuffer_tx[2];
static led_pixel_column_t g_ledbuffer_blank_tx;
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
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wtautological-compare"
  _Static_assert(&g_ledbuffer_tx->d0 == &g_ledbuffer_tx->channel[0]);
#if FOUR_CHANNEL
  _Static_assert(&g_ledbuffer_tx->d90i == &g_ledbuffer_tx->channel[1]);
  _Static_assert(&g_ledbuffer_tx->d180 == &g_ledbuffer_tx->channel[2]);
  _Static_assert(&g_ledbuffer_tx->d270i == &g_ledbuffer_tx->channel[3]);
#else
  _Static_assert(&g_ledbuffer_tx->d180i == &g_ledbuffer_tx->channel[1]);
#endif
#pragma GCC diagnostic pop

  // Initialize g_ledbuffer_tx
  for (size_t b = 0; b != sizeof(g_ledbuffer_tx) / sizeof(*g_ledbuffer_tx); ++b) {
    for (size_t c = 0; c != 4; ++c) {
      g_ledbuffer_tx[b].channel[c].sof = 0;
      // Extra clocks to ensure all leds are set
      for(size_t i = 0; i != LEDS_DRIVER_EOF_PIXELS; ++i) {
        g_ledbuffer_tx[b].channel[c].eof[i] = 0x000000E1;
      }
    }
    leds_driver_update_hue(0);
  }

  // Initialize g_ledbuffer_blank_tx
  g_ledbuffer_blank_tx.sof = 0;
  for (size_t i = 0; i < LEDS_Y; ++i) {
    g_ledbuffer_blank_tx.pixel[i] = 0x000000E1; // pretty much turn off fully...
  }
  for(size_t i = 0; i != LEDS_DRIVER_EOF_PIXELS; ++i) {
    g_ledbuffer_blank_tx.eof[i] = 0x000000E1;
  }

  g_ledbuffer_tx_sent = 0;

  // Prepare DMA
#define TRANSFER_DIVIDE(peripheral, value) (LL_SPI_GetDataWidth(peripheral) == LL_SPI_DATAWIDTH_8BIT ? (value) : (value / 2))
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS1), LL_SPI_DMA_GetRegAddr(hLEDS1));
  LL_DMA_SetDataLength(TXDMA(hLEDS1), TRANSFER_DIVIDE(hLEDS1, sizeof(led_pixel_column_t)));
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS3), LL_SPI_DMA_GetRegAddr(hLEDS3));
  LL_DMA_SetDataLength(TXDMA(hLEDS3), TRANSFER_DIVIDE(hLEDS3, sizeof(led_pixel_column_t)));
#if FOUR_CHANNEL
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS2), LL_SPI_DMA_GetRegAddr(hLEDS2));
  LL_DMA_SetDataLength(TXDMA(hLEDS2), TRANSFER_DIVIDE(hLEDS2, sizeof(led_pixel_column_t)));
  LL_DMA_SetPeriphAddress(TXDMA(hLEDS4), LL_SPI_DMA_GetRegAddr(hLEDS4));
  LL_DMA_SetDataLength(TXDMA(hLEDS4), TRANSFER_DIVIDE(hLEDS4, sizeof(led_pixel_column_t)));
#endif
#undef TRANSFER_DIVIDE

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

led_data_buffer_t* leds_driver_get_next_buffer(void) {
  size_t next = (g_ledbuffer_tx_sent + 1) % 2;
  return &(g_ledbuffer_tx[next]);
}

void leds_driver_update_hue(uint16_t hue) {
  const uint32_t LEDBASE = 0x000000E0 | (g_brightness >> 3);

  led_data_buffer_t* b = leds_driver_get_next_buffer();

  for (size_t s = 0; s < 4; ++s) {
    uint16_t hx = hue + s * 0x4000;
    uint32_t* const pixel = b->channel[s].pixel;

    for (size_t i = 0; i < LEDS_Y; ++i) {
      pixel[i] =
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

uint8_t leds_driver_get_brightness(void) {
  return g_brightness;
}

void leds_driver_set_brightness(uint8_t brightness) {
  g_brightness = brightness;
}

void leds_driver_transmit(void) {
  size_t b = g_ledbuffer_tx_sent;

  LL_DMA_SetMemoryAddress(TXDMA(hLEDS1), (uint32_t)&(g_ledbuffer_tx[b].d0));
#if FOUR_CHANNEL
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS3), (uint32_t)&(g_ledbuffer_tx[b].d180));
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS2), (uint32_t)&(g_ledbuffer_tx[b].d90i));
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS4), (uint32_t)&(g_ledbuffer_tx[b].d270i));
#else
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS3), (uint32_t)&(g_ledbuffer_tx[b].d180i));
#endif

  LL_DMA_EnableStreamEx(TXDMA(hLEDS1));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS3));
#if FOUR_CHANNEL
  LL_DMA_EnableStreamEx(TXDMA(hLEDS2));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS4));
#endif

  g_ledbuffer_tx_sent = (b + 1) % 2;
}

void leds_driver_transmit_blank(void) {
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS1), (uint32_t)&g_ledbuffer_blank_tx);
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS3), (uint32_t)&g_ledbuffer_blank_tx);
#if FOUR_CHANNEL
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS2), (uint32_t)&g_ledbuffer_blank_tx);
  LL_DMA_SetMemoryAddress(TXDMA(hLEDS4), (uint32_t)&g_ledbuffer_blank_tx);
#endif

  LL_DMA_EnableStreamEx(TXDMA(hLEDS1));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS3));
#if FOUR_CHANNEL
  LL_DMA_EnableStreamEx(TXDMA(hLEDS2));
  LL_DMA_EnableStreamEx(TXDMA(hLEDS4));
#endif
}
