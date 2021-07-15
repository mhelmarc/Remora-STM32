/*
 * dmaspi.h
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */
#ifndef DMASPI_H_
#define DMASPI_H_
#include "config/spi.h"
#include "spidma.h"
#include "configuration.h"
#include "remora.h"

using namespace stm32plus;


using vPFunction = void (*)(volatile rxData_t*);

//maybe a template would be good here
class SPIDma {

  private:

    enum {
      BUFFER_SIZE = SPI_BUFF_SIZE,
    };

    SPI_t *_spi;
    SpiDmaTxChannel_t _spi_txdma;
    SpiDmaRxChannel_t _spi_rxdma;
    DMAStreamCopier1_t _stream_copier1;
//    DMAStreamCopier2_t _stream_copier_two;

    volatile txData_t *_txData;
    volatile rxData_t *_rxData;
    volatile rxData_t *_RxBuffer;    // pointer to current buffer used
    volatile rxData_t spiRxBuffer1;  // this buffer is used to check for valid data before moving it to rxData
    volatile rxData_t spiRxBuffer2;  // this buffer is used to check for valid data before moving it to rxData

    void setRxTxDmaEnableBits(void);
    void onDmaTxInterrupt(DmaEventType ev);
    void onDmaRxInterrupt(DmaEventType ev);
    void onDmaStreamCopier1Interrupt(DmaEventType ev);
//    void onDmaStreamCopier2Interrupt(DmaEventType ev);

    void disableDmaInterrupts(void);
    void enableDmaInterrupts(void);

    vPFunction _spi_DmaRxInterruptCallback;

  public:
    SPIDma(volatile txData_t*, volatile rxData_t*, vPFunction);
    ~SPIDma();

    volatile int32_t buff1_header, buff2_header; // this should be a pointer ??

    void init(void);
    void enableStreamCopier(volatile rxData_t*);
    void disableStreamCopier(void);
    void disableRxDmaStream(void);
    void enableRxDmaStream(void);
    void swapBuffer(volatile rxData_t*);

    volatile bool isCopyComplete;

};

#endif /* DMASPI_H_ */
