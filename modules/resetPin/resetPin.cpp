#include "resetPin.h"

ResetPin::ResetPin(volatile bool &ptrReset, GpioPinRefs& pin) :
    ptrReset(&ptrReset), pin(pin) {
  /* Pin used for this is an INPUT, mode is already set before calling this class */
}

void ResetPin::update() {

  *(this->ptrReset) = this->pin.read();

}

void ResetPin::slowUpdate() {
  return;
}
