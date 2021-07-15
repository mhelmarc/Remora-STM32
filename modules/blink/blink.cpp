#include "blink.h"

Blink::Blink(GpioPinRefs& Pin, uint32_t threadFreq,
    uint32_t freq){

  this->periodCount = threadFreq / freq;
  this->blinkCount = 0;
  this->bState = false;
  this->blinkPin = Pin;
  this->blinkPin.setState(bState);
}


void Blink::update(void) {
  ++this->blinkCount;
  if (this->blinkCount >= this->periodCount / 2) {
//    this->blinkPin.setState(this->bState = !this->bState);
    this->blinkPin.setState(this->bState ^= true);
    this->blinkCount = 0;
  }
}


void Blink::slowUpdate(void) {
  return;
}
