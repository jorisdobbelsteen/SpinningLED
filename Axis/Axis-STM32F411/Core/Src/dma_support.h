//
// Created by Joris on 29/01/2024.
//

#ifndef AXIS_DMA_SUPPORT_H
#define AXIS_DMA_SUPPORT_H

#include "main.h"

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

#endif //AXIS_DMA_SUPPORT_H
