#ifndef THREAD_PRUTIMER_H_
#define THREAD_PRUTIMER_H_

#include "timers.h"
#include <cmath>

template<typename TTimer>
class PRUThread; // forward declatation


/**
 *  PRU Timer
 *  @tparam TTimer timer with defined interrupt feature
 */

template<typename TTimer>
class PRUTimer {

  private:
    enum {
      TIMER_BASE_FREQUENCY = 1000000,
    };

    PRUThread<TTimer> *timerOwnerPtr;

    TTimer timer;
    uint32_t frequency;

    void timerTick();
    void onInterrupt(TimerEventType, uint8_t);

  public:
    PRUTimer(uint32_t, uint8_t, uint8_t, PRUThread<TTimer>*);
};


// Constructor
template<typename TTimer>
PRUTimer<TTimer>::PRUTimer(uint32_t freq, uint8_t priority, uint8_t subPriority,
    PRUThread<TTimer>* ownerPtr) : timerOwnerPtr(ownerPtr), frequency(freq)  {

  uint16_t period;
  float f = static_cast<float>(frequency);
  period = floorf(TIMER_BASE_FREQUENCY / f);

  this->timer.setTimeBaseByFrequency(TIMER_BASE_FREQUENCY, period - 1);

  this->timer.TimerInterruptEventSender.insertSubscriber(
      TimerInterruptEventSourceSlot::bind(this, &PRUTimer::onInterrupt));

  this->timer.setNvicPriorities(priority, subPriority);

  this->timer.enablePeripheral();
  this->timer.enableInterrupts(TIM_IT_Update);

}


template<typename TTimer>
inline void PRUTimer<TTimer>::timerTick() {
  this->timerOwnerPtr->run();
}


template<typename TTimer>
void PRUTimer<TTimer>::onInterrupt(TimerEventType tev,
    uint8_t t/* timerNumber */) {
  (void)(t); //silence compiler
  if (tev == TimerEventType::EVENT_UPDATE) {
    this->timerTick();
  }
}


#endif /* THREAD_PRUTIMER_H_ */
