#ifndef PRUTHREAD_H_
#define PRUTHREAD_H_

#include "modules/module.h"
#include "pruTimer.h"

// Standard Template Library (STL) includes
#include <iostream>
#include <vector>

using namespace std;


/**
 *  PRU Timer
 *  @tparam TTimer timer with defined interrupt feature
 */

template<typename TTimer>
class PRUThread {

  private:

    PRUTimer<TTimer> *TimerPtr;

    uint32_t frequency;
    uint8_t priority;   // interrupt priorities
    uint8_t subpriority;

    vector<Module*> vThread;     // vector containing pointers to Thread modules
    vector<Module*>::iterator iter ;

  public:
    PRUThread(uint32_t, uint8_t, uint8_t);

    void registerModule(Module *module);
    void startThread(void);
    void run(void);
};

// Constructor
template<typename TTimer>
PRUThread<TTimer>::PRUThread(uint32_t freq, uint8_t irqpriority, uint8_t irqsubpriority)
    : frequency(freq), priority(irqpriority), subpriority(irqsubpriority) {
  TimerPtr = NULL;
  iter = 0;

}


template<typename TTimer>
void PRUThread<TTimer>::startThread(void) {
  TimerPtr = new PRUTimer<TTimer>(this->frequency, this->priority,
      this->subpriority, this);
}


template<typename TTimer>
void PRUThread<TTimer>::registerModule(Module* module) {
  this->vThread.push_back(module);
}


template<typename TTimer>
void PRUThread<TTimer>::run(void) {
//   iterate over the Thread pointer vector to run all instances of Module::runModule()
  for (iter = vThread.begin(); iter != vThread.end(); ++iter) {
    (*iter)->runModule();
  }
}
#endif /* PRUTHREAD_H_ */
