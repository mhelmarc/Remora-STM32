#include "digitalPin.h"

DigitalPin::DigitalPin(volatile uint8_t &ptrData, int mode, GpioPinRefs& pin,
    int bitNumber, bool invert) : ptrData(&ptrData), mode(mode), bitNumber(bitNumber),
    invert(invert) {
  this->mask = 1 << this->bitNumber;
  this->pin = pin;
}

void DigitalPin::update() {
  bool pinState;

  if (this->mode == 0)							// the pin is configured as an input
      {
    pinState = this->pin.read();
    if (this->invert) {
      pinState = !pinState;
    }

    if (pinState == 1)							// input is high
        {
      *(this->ptrData) |= this->mask;
    }
    else										// input is low
    {
      *(this->ptrData) &= ~this->mask;
    }
  }
  else											// the pin is configured as an output
  {
    pinState = *(this->ptrData) & this->mask;	// get the value of the bit in the data source
    if (this->invert) {
      pinState = !pinState;
    }
    this->pin.setState(pinState);			    // simple conversion to boolean
  }
}

void DigitalPin::slowUpdate() {
  return;
}
