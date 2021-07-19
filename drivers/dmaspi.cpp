/*
 * dmaspi.cpp
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */

#include "config/stm32plus.h"
#include "config/dma.h"
#include "config/spi.h"
#include "dmaspi.h"

// Constructor
SPIDma::SPIDma(volatile txData_t *txdata, volatile rxData_t *rxdata,
    vPFunction fp) :
    _txData(txdata), _rxData(rxdata), _spi_DmaRxInterruptCallback(fp) {

  buff1_header = spiRxBuffer1.data.header;
  buff2_header = spiRxBuffer2.data.header;

  isCopyComplete = false;
  _RxBuffer = &spiRxBuffer1;
  SPI_t::Parameters params; //TODO move this to main like the serialcom
  /* Set the default parameters */
  params.spi_direction = SPI_Direction_2Lines_FullDuplex;
  params.spi_mode = SPI_Mode_Slave;
  params.spi_dataSize = SPI_DataSize_8b;
  params.spi_cpol = SPI_CPOL_Low; /* SCLK polarity and phase MUST be the same as the Master for this to work */
  params.spi_cpha = SPI_CPHA_1Edge;
  params.spi_baudRatePrescaler = SPI_BaudRatePrescaler_2; /* Not needed in slave mode ??? */
  params.spi_firstBit = SPI_FirstBit_MSB;
  params.spi_polynomial = 7; /* CRC? */

  _spi = new SPI_t(params);
  _spi->enablePeripheral();

  _spi_rxdma.setNvicPriorities(1, 0);
  _spi_txdma.setNvicPriorities(1, 0);
  _stream_copier1.setNvicPriorities(1,0);

  /* Setup DMA transmit/receive interrupts */
  _spi_rxdma.DmaInterruptEventSender.insertSubscriber(
      DmaInterruptEventSourceSlot::bind(this, &SPIDma::onDmaRxInterrupt));

  _spi_txdma.DmaInterruptEventSender.insertSubscriber(
      DmaInterruptEventSourceSlot::bind(this, &SPIDma::onDmaTxInterrupt));

  _stream_copier1.DmaInterruptEventSender.insertSubscriber(
      DmaInterruptEventSourceSlot::bind(this,
          &SPIDma::onDmaStreamCopier1Interrupt));
}


SPIDma::~SPIDma() {
  delete _spi;
}

/* There seems to be a sequence for the DMA to work
 * If DMA streams is initialize first, the Spi RX/TX DMAEN bits doesn't get enabled.
 * If the peripheral is initialize first, the DMA FIFO interrupt error kicks in.
 * Since we're not using FIFO mode I guess it's OK ??
 * The library does not have a provision to explicitly enable RX/TX DMAEN bits
 * We'll use direct call to stdperiph lib, but defeats my attempt to generalize
 * this in case I switch to a different SPI number
 */
inline void SPIDma::setRxTxDmaEnableBits(void) {
  SPI_I2S_DMACmd(SPI1,(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN), ENABLE);
}



void SPIDma::init(void) {

  disableDmaInterrupts();

  isCopyComplete = false;

  _stream_copier1.disableStream();
  _spi_txdma.disableStream();
  _spi_rxdma.disableStream();

  setRxTxDmaEnableBits();

  _RxBuffer = &spiRxBuffer1;

  /* Streams are enabled with these calls */
  _spi_txdma.beginWrite(const_cast<uint8_t*>(_txData->tx.buffer), BUFFER_SIZE);
  _spi_rxdma.beginRead(const_cast<uint8_t*>(_RxBuffer->rx.buffer),
      BUFFER_SIZE);

  enableDmaInterrupts();

  /* Clearing fifo error is always been available :-) */
  _spi_txdma.clearErrorFlag();
  _spi_rxdma.clearErrorFlag();
}

/* Can't seem to step thru the register to check for DMAEN bit
 * it always stay at 0, but the interrupt still fires and works ok
 * so it's good.
 * I'm perflexed, how did this worked before when I was copying the
 * whole struct instead of just the buffers?
 */
void SPIDma::enableStreamCopier(volatile rxData_t *src) {
  _stream_copier1.beginCopyMemory(const_cast<uint8_t*>(_rxData->rx.buffer), const_cast<uint8_t*>(src->rx.buffer),
      BUFFER_SIZE, DMA_Priority_Medium);
  enableRxDmaStream();
}


/* Disable the memory copy DMA stream */
void SPIDma::disableStreamCopier(void) {
  _stream_copier1.disableStream();
}


/* Disable the recieve DMA stream */
void SPIDma::disableRxDmaStream(void) {
  _spi_rxdma.disableStream();
}


/* Enable the receive DMA stream */
inline void SPIDma::enableRxDmaStream(void) {
  _spi_rxdma.enableStream();
}


/* Enable interrupts */
inline void SPIDma::enableDmaInterrupts(void) {
  _spi_txdma.enableInterrupts(SpiDmaRxTransferComplete);
  _spi_rxdma.enableInterrupts(SpiDmaTxTransferComplete);
  _stream_copier1.enableInterrupts(StreamCopier1TransferComplete);
}


/* Disable interrupt */
inline void SPIDma::disableDmaInterrupts(void) {
  _spi_txdma.disableInterrupts(SpiDmaRxTransferComplete);
  _spi_rxdma.disableInterrupts(SpiDmaTxTransferComplete);
  _stream_copier1.disableInterrupts(StreamCopier1TransferComplete);
}

/* This is called from the callback function in main */
void SPIDma::swapBuffer(volatile rxData_t* curbuf) {
  if (curbuf == &spiRxBuffer1){
    _RxBuffer = &spiRxBuffer2;
  }
  else if (curbuf == &spiRxBuffer2) {
    _RxBuffer = &spiRxBuffer1;
  }
}


/** DMA Receive interrupt
 */
void SPIDma::onDmaRxInterrupt(DmaEventType ev) {

  if (ev == DmaEventType::EVENT_COMPLETE) {
    /* call the function in main */
    _spi_DmaRxInterruptCallback(_RxBuffer);

    /* Interrupts automatically disables the streams
     * it is re-initialized when beginread()is called ,
     */
    /* No need to check if the transfer is complete, we got here because it was completed */
    _spi_rxdma.clearCompleteFlag();
    _spi_rxdma.beginRead(const_cast<uint8_t*>(_RxBuffer->rx.buffer),
        BUFFER_SIZE);
  }
}


/* DMA Transmit interrupt */
void SPIDma::onDmaTxInterrupt(DmaEventType ev) {

  if (ev == DmaEventType::EVENT_COMPLETE) {
    /* No need to check if the transfer is complete, we got here because it was completed */
    _spi_txdma.clearCompleteFlag();
    _spi_txdma.beginWrite(const_cast<uint8_t*>(_txData->tx.buffer),
        BUFFER_SIZE);
  }
}


/* DMA Stream copier one */
void SPIDma::onDmaStreamCopier1Interrupt(DmaEventType ev) {
  if (ev == DmaEventType::EVENT_COMPLETE) {
    /* No need to check if the transfer is complete, we got here because it was completed */
    _stream_copier1.clearCompleteFlag();
    isCopyComplete = true;
  }
}
