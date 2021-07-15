/*
 *                  MAX_CLK(MHz)    APB1_CLK(MHz)   APB2_CLK(MHz)
 * STM32F401        84              42              84
 * STM32F411        100             50              100
 * STM32F405        168             42              84
 * STM32F407        168             42              84
 *
 * On STM32F407:
 * 16bit Timers 1,8 &  9:11 on APB2 PCLK of 84MHz
 * 16/32bit Timers 2:5      on APB1 PCLK of 42MHz
 * 16bit Timers 6:7 & 12:14 on APB1 PCLK of 42MHz
 */

#ifndef TIMERS_H_
#define TIMERS_H_

#include "config/timing.h"

using namespace stm32plus;

typedef Timer3<
    Timer3InternalClockFeature, /* Internal clock as source */
    Timer3InterruptFeature      /* It's going to be a source of interrupt */
> BaseThreadTimer_t;

typedef Timer4<
  Timer4InternalClockFeature,   /* Internal clock as source */
  Timer4InterruptFeature        /* It's going to be a source of interrupt */
> ServoThreadTimer_t;

#endif /* TIMERS_H_ */
