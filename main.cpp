/*
 * main.cpp
 *    This file is part of Remora-STM32 a port of Remora
 *    (https://github.com/scottalford75/Remora) a free, opensource
 *    LinuxCNC component and Programmable Realtime Unit (PRU)
 *
 *    Copyright (C) 2021
 *
 *    This program is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU General Public License version 2
 *    of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

#include "config/stm32plus.h"
#include "utils/wd.h"
#include "drivers/dmaspi.h"
#include "drivers/serialcom.h"

#include "configuration.h"
#include "remora.h"

#include "thread/pruThread.h"
#include "modules/module.h"

/***********************************************************************
 *                STRUCTURES AND GLOBAL VARIABLES                       *
 ************************************************************************/

typedef Usart1<Usart1InterruptFeature> SerialPort_t;
SerialCom<SerialPort_t> *_serial;


// state machine
enum State {
  ST_SETUP = 0,
  ST_START,
  ST_IDLE,
  ST_RUNNING,
  ST_STOP,
  ST_RESET,
  ST_WDRESET
};

uint8_t resetCnt;
volatile uint8_t rejectCnt;

// boolean
static volatile bool SPIdata;
static volatile bool SPIdataError;
static volatile bool PRUreset;
bool configError = false;
bool threadsRunning = false;

// pointers to objects with global scope
PRUThread<ServoThreadTimer_t> *servoThread;
PRUThread<BaseThreadTimer_t> *baseThread;
// DMA Controller
SPIDma *_spiDma;

volatile rxData_t rxData;
volatile txData_t txData;


void uputcWrapper(uint8_t d) {
  _serial->uputc(d);
}


uint8_t ugetcWrapper(void) {
  return _serial->ugetc();
}


/* This is called from SPI dma receive complete interrupt */
void spiRxCompleteCallback(volatile rxData_t *rx) {

  _spiDma->disableRxDmaStream();

  SPIdata = false;
  SPIdataError = false;

  switch (rx->data.header) {
    case PRU_READ:
      SPIdata = true;
      rejectCnt = 0;
      _spiDma->disableStreamCopier();
    break;

    case PRU_WRITE:
      SPIdata = true;
      rejectCnt = 0;
      _spiDma->enableStreamCopier(rx); // copy the current buffer
    break;

    default:
      rejectCnt++;
      if (rejectCnt > 5) {
        SPIdataError = true;
      }
      _spiDma->disableStreamCopier();
  }

  _spiDma->swapBuffer(rx);

}


void DMAsetup() {
  _spiDma = new SPIDma(&txData, &rxData, &spiRxCompleteCallback);
  _spiDma->init();
}


void DMAreset() {
  _spiDma->init();
}


void setup() {

  xputs("\n1. Setting up DMA and threads\n");

  DMAsetup();

  txData.data.header = PRU_DATA;

  // Note: DMA has highest priority, then Base thread and then Servo thread
  //       to ensure SPI data transfer is reliable

  baseThread = new PRUThread<BaseThreadTimer_t>(PRU_BASEFREQ, 2, 0);
  servoThread = new PRUThread<ServoThreadTimer_t>(PRU_SERVOFREQ, 3, 0);

  // Other interrupt sources
  // TBA

}

/* Configurations are sent through the serial port
 * A dedicated gpio pin should maybe used to enter configuration mode
 * instead of a special character
 */
int loadModules() {
  xputs("\n2. Load Configuration\n");
  char ch;
  while ((ch = _serial->ugetc()) != '$')
    ;

  if (setConfiguration(&PRUreset) < 0) { //pseudo check
    xputs("Error: data incomplete\n");
    return -1;
  }
  return 0;
}


void initSerialCom(void) {

  SerialPort_t::Parameters params;
  params.usart_baudRate=PC_BAUD;
  params.usart_wordLength=USART_WordLength_8b;
  params.usart_parity=USART_Parity_No;
  params.usart_stopBits=USART_StopBits_1;
  params.usart_flowControl=USART_HardwareFlowControl_None;
  params.usart_mode=USART_Mode_Rx | USART_Mode_Tx; // trasnmit and receive is used
  params.usart_synchronous=false;

  _serial = new SerialCom<SerialPort_t>(params);

  xdev_out(uputcWrapper); // xprintf pointer to our serial in/out function
  xdev_in(ugetcWrapper);

  /* Interrupts won't work until NVIC is initialize
   * We're using default group priority of 4, all bits
   * are used for priority no bits left for subpriority
   */
  Nvic::initialise(); // needed for the interrupts

  MillisecondTimer::initialise(); // needed for delays

  MillisecondTimer::delay(1);

}


/*
 * Main entry point
 */

int main() {

  enum State currentState;
  enum State prevState;

  SPIdata = false;
  SPIdataError = false;
  currentState = ST_SETUP;
  prevState = ST_RESET;

  initSerialCom(); // setup USART early we need it for printf

  xprintf("\nRemora-STM32 - Programmable Realtime Unit\n\n");

  WatchDog::start(2000);

  while (1) {
    // the main loop does very little, keeping the Watchdog serviced and
    // resetting the rxData buffer if there is a loss of SPI commmunication
    // with LinuxCNC. Everything else is done via DMA and within the
    // two threads- Base and Servo threads that run the Modules.

    WatchDog::feed();

    switch (currentState) {
      case ST_SETUP:
        // do setup tasks

        if (currentState != prevState) {
          xputs("## Entering SETUP state\n");
        }
        prevState = currentState;
        setup();
        loadModules();
        currentState = ST_START;
      break;

      case ST_START:
        // do start tasks
        if (currentState != prevState) {
          xputs("\n## Entering START state\n");
        }
        prevState = currentState;

        if (!threadsRunning) {
          // Start the threads
          xputs("\nStarting the BASE thread\n");
          baseThread->startThread();

          xputs("\nStarting the SERVO thread\n");
          servoThread->startThread();

          threadsRunning = true;

          // wait for threads to read IO before testing for PRUreset
//              wait(1);
          MillisecondTimer::delay(1);
        }

        if (PRUreset) {
          // RPi outputs default is high until configured when LinuxCNC spiPRU component is started, PRUreset pin will be high
          // stay in start state until LinuxCNC is started
          currentState = ST_START;
        }
        else {
          currentState = ST_IDLE;
        }

      break;

      case ST_IDLE:
        // do something when idle
        if (currentState != prevState) {
          xputs("\n## Entering IDLE state\n");
        }
        prevState = currentState;

        // check to see if there there has been SPI errors
        if (SPIdataError) {
          xputs("SPI data error:\n");
          xprintf("    spiRxBuffer1.header = %x\n", _spiDma->buff1_header);
          xprintf("    spiRxBuffer2.header = %x\n", _spiDma->buff2_header);
          SPIdataError = false;
        }

        //wait for SPI data before changing to running state
        if (SPIdata) {
          currentState = ST_RUNNING;
        }

        if (PRUreset) {
          currentState = ST_WDRESET;
        }

      break;

      case ST_RUNNING:
        // do running tasks
        if (currentState != prevState) {
          xputs("\n## Entering RUNNING state\n");
        }
        prevState = currentState;

        // check to see if there there has been SPI errors
        if (SPIdataError) {
          xputs("SPI data error:\n");
          xprintf("    spiRxBuffer1.header = %x\n", _spiDma->buff1_header);
          xprintf("    spiRxBuffer2.header = %x\n", _spiDma->buff2_header);
          SPIdataError = false;
        }

        if (SPIdata) {
          // SPI data received by DMA
          resetCnt = 0;
          SPIdata = false;
        }
        else {
          // no SPI data received by DMA
          resetCnt++;
        }

        if (resetCnt > SPI_ERR_MAX) {
          // reset threshold reached, reset the PRU
          xputs("   SPI data error limit reached, resetting\n");
          resetCnt = 0;
          currentState = ST_RESET;
        }

        if (PRUreset) {
          currentState = ST_WDRESET;
        }

      break;

      case ST_STOP:
        // do stop tasks
        if (currentState != prevState) {
          xputs("\n## Entering STOP state\n");
        }
        prevState = currentState;

        currentState = ST_STOP;
      break;

      case ST_RESET:
        // do reset tasks
        if (currentState != prevState) {
          xputs("\n## Entering RESET state\n");
        }
        prevState = currentState;

        // reset DMA transfers - this does not work properly... :-( ??
//          DMAreset(); // as said it does not work, well I tried to be faithful to the origintal :-)

        // set all of the rxData buffer to 0
        // rxData.rx.buffer is volatile so need to do this the long way. memset cannot be used for volatile
        xputs("   Resetting rxBuffer\n");
        {
          int n = sizeof(rxData.rx.buffer);
          while (n-- > 0) {
            rxData.rx.buffer[n] = 0;
          }
        }

        currentState = ST_IDLE;
      break;

      case ST_WDRESET:
        // do a watch dog reset
        xputs("\n## Entering WDRESET state\n");

        // force a watchdog reset by looping here
        while (1) {
        }

      break;
    }

    MillisecondTimer::delay(LOOP_TIME);

  }
}
