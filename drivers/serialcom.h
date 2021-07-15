/*
 * uart.h
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 *  Derived from the works of ChaN http://www.elm-chan.org/
 */

#ifndef SERIAL_COM_H_
#define SERIAL_COM_H_

#include "config/usart.h"
#include "utils/xprintf.h"

using namespace stm32plus;

template<typename TUsart>
class SerialCom{
  private:

    TUsart *_usart;

    typedef typename TUsart::Parameters Param_t;

    enum {
      BUFFER_SIZE = 128
    };

    volatile struct {
        uint16_t tri, twi, tct;
        uint16_t rri, rwi, rct;
        uint8_t tbuf[BUFFER_SIZE];
        uint8_t rbuf[BUFFER_SIZE];
    } Fifo;

    void onInterrupt(UsartEventType);

  public:
    SerialCom(Param_t);
    ~SerialCom();

    uint8_t ugetc(void);
    void uputc(uint8_t);
    int dataReady(void);
};



/**
 * Constructor
 */
template<typename TUsart>
SerialCom<TUsart>::SerialCom(Param_t params){

  this->Fifo.tct = 0;
  this->Fifo.twi = 0;
  this->Fifo.tri = 0;
  this->Fifo.rct = 0;
  this->Fifo.rri = 0;
  this->Fifo.rwi = 0;

  /* actual usart initialization */
  this->_usart = new TUsart(params);

  this->_usart->UsartInterruptEventSender.insertSubscriber(
      UsartInterruptEventSourceSlot::bind(this, &SerialCom::onInterrupt));
  this->_usart->enableInterrupts(USART_IT_TXE);
  this->_usart->enableInterrupts( USART_IT_RXNE);
  this->_usart->clearPendingInterruptsFlag(USART_IT_TC);
}



/**
 * Destructor
 */
template<typename TUsart>
SerialCom<TUsart>::~SerialCom() {
  delete this->_usart;
}



/* Check if the index count is >= 1 */
template<typename TUsart>
int SerialCom<TUsart>::dataReady(void) {
  return this->Fifo.rct;
}



/* Get a single character from the input buffer */
template<typename TUsart>
uint8_t SerialCom<TUsart>::ugetc(void)
{
  uint8_t d;
  int i;

  if (!this->Fifo.rct) return 0;
  /* Wait while rx fifo is empty */
//  while (!this->Fifo.rct);

  i = this->Fifo.rri; /* Get a byte from rx fifo */
  d = this->Fifo.rbuf[i];
  this->Fifo.rri = ++i % this->BUFFER_SIZE;

  this->_usart->disableInterrupts(USART_IT_RXNE);
  this->Fifo.rct--;
  this->_usart->enableInterrupts(USART_IT_RXNE);

  return d;
}



/* Transmit one character at a time
 * actual transmission happens in the interrupt routine
 * we just fill the output buffer here
 */
template<typename TUsart>
void SerialCom<TUsart>::uputc(uint8_t d) {
  int i;
  /* Wait if buffer is full */
  while (this->Fifo.tct >= this->BUFFER_SIZE) ;

  i = this->Fifo.twi; /* Put a byte into Tx fifo */
  this->Fifo.tbuf[i] = d;
  this->Fifo.twi = ++i % this->BUFFER_SIZE;
//  Nvic::disableAllInterrupts();
  this->Fifo.tct++;
  this->_usart->enableInterrupts(USART_IT_TXE);
//  Nvic::enableAllInterrupts();
}



/* Interrupts */
template<typename TUsart>
void SerialCom<TUsart>::onInterrupt(UsartEventType uev) {

  uint8_t d;
  int i;

  /* Receive Interrupt */
  if (uev == UsartEventType::EVENT_RECEIVE) {
    /* Get received byte */
    d = this->_usart->receive();
    i = this->Fifo.rct;

    if (i < this->BUFFER_SIZE) { /* Store it into the rx fifo if not full */
      this->Fifo.rct = ++i;
      i = this->Fifo.rwi;
      this->Fifo.rbuf[i] = d;
      this->Fifo.rwi = ++i % this->BUFFER_SIZE;
    }
  }


  /* Transmit Interrupt */
  if (uev == UsartEventType::EVENT_READY_TO_TRANSMIT) {
    i = this->Fifo.tct;
    if (i--) { /* There is any data in the tx fifo */
      this->Fifo.tct = static_cast<uint16_t>(i);
      i = this->Fifo.tri;
      this->_usart->send(this->Fifo.tbuf[i]);
      this->Fifo.tri= ++i % this->BUFFER_SIZE;
    }
    else { /* No data in the tx fifo */
      this->_usart->disableInterrupts(USART_IT_TXE);
    }
  }
}

#endif /* SERIAL_COM_H_ */



