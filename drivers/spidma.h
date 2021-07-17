/*
 * spidma.h
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */
#ifndef SPIDMA_H_
#define SPIDMA_H_

using namespace stm32plus;

/* SPI1 on Port A4:A7
 *
 * SPI1_NSS: PA4
 * SPI1_SCK: PA5
 * SPI1_MiSO: PA6
 * SPI1_MOSI: PA7
 * SPI1 is on APB2 which is 2 times slower than the CPU clock
 * in the case of F407, 168MHz/2=84MHz, SPI lowest divider is 2.
 * The maximum SPI1 frequency we can get is 42MHz
 */
#define SpiDmaRxTransferComplete Spi1TxDmaChannelInterruptFeature::COMPLETE
#define SpiDmaTxTransferComplete Spi1RxDmaChannelInterruptFeature::COMPLETE
#define StreamCopier1TransferComplete Dma2Stream7InterruptFeature::COMPLETE
//#define StreamCopier2TransferComplete Dma2Stream0InterruptFeature::COMPLETE

/* SPI1 transmit is on DMA2 Channel 3 Stream 3 */
typedef Spi1TxDmaChannel<
    Spi1TxDmaChannelInterruptFeature, /* We'll use interrupt */
    SpiDmaWriterFeature<
      Spi1PeripheralTraits,
      DMA_Priority_VeryHigh,
      true,
      DMA_FIFOMode_Disable
    >
> SpiDmaTxChannel_t;

/* Receive on DMA2 Chanel 3 Stream 0 */
typedef Spi1RxDmaChannel<
    Spi1RxDmaChannelInterruptFeature, /* We'll use interrupt */
    SpiDmaReaderFeature<
      Spi1PeripheralTraits,
      DMA_Priority_High,
      true,
      DMA_FIFOMode_Disable
    >
> SpiDmaRxChannel_t;

/* Memory to memory DMA transfer is only available on DMA2 */
typedef Dma2Channel0Stream7<
    Dma2Stream7InterruptFeature,
    DmaMemoryCopyFeature<>          /* memory copy with default transfer size (bytes) */
> DMAStreamCopier1_t;

//typedef Dma2Channel1Stream0<
//    Dma2Stream0InterruptFeature,    // interrupts on DMA2, stream 0
//    DmaMemoryCopyFeature<>          // memory copy with default transfer size (bytes)
//> DMAStreamCopier2_t;

#endif /* SPIDMA_H_ */
