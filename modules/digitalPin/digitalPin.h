#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include "modules/module.h"
#include "drivers/GpioPinRefs.h"

class DigitalPin: public Module {
  private:

    volatile uint8_t *ptrData; 	// pointer to the data source
    int mode;                   // input or output, only used for condition checking
                                // actual mode is setup before calling this class
    GpioPinRefs pin;
    int bitNumber;				// location in the data source
    bool invert;
    int mask;

  public:

    DigitalPin(volatile uint8_t&, int, GpioPinRefs&, int, bool);
    virtual void update(void);
    virtual void slowUpdate(void);
};

#endif
