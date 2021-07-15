/*
 * wd.h
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 */

#ifndef WD_H_
#define WD_H_

#include "config/stm32plus.h"
#include "config/stdperiph.h"

class WatchDog {
  public:
    static void start(uint16_t);
    static void feed();
};


void WatchDog::start(uint16_t timeout) {

  uint16_t reload;

  /* Check if IWDG causes the reset */
  if (RCC_GetFlagStatus(RCC_FLAG_IWDGRST) != RESET) {
    /* Clear the reset flag */
    RCC_ClearFlag();
  }

  IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable); // IWDG->KR = 0x5555

  if (timeout < 8) {
    reload = 8;
  } else if (timeout > 32768){
    reload = 32768;
  } else {
    reload = timeout;
  }

  /* Set the divider for the Low Speed Internal clock */
  IWDG_SetPrescaler(0x07); // LSI Divider = 256 ticks @128Hz

  IWDG_SetReload(reload);

//  IWDG_ReloadCounter();

  IWDG_Enable();
}

/* Feed the dog or else system will reset */
void WatchDog::feed(void) {
  IWDG_ReloadCounter();
}



#endif /* WD_H_ */
