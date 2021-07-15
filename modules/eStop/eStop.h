#ifndef ESTOP_H
#define ESTOP_H

#include "modules/module.h"
#include "drivers/GpioPinRefs.h"
#include "../../configuration.h"


class eStop: public Module {

  private:

    volatile int32_t *ptrTxHeader;
    GpioPinRefs pin;
  public:

    eStop(volatile int32_t&, GpioPinRefs&);

    virtual void update(void);
    virtual void slowUpdate(void);
};

#endif
