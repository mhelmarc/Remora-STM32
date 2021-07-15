#ifndef MODULES_RESETPIN_H_
#define MODULES_RESETPIN_H_

#include "modules/module.h"
#include "drivers/GpioPinRefs.h"

class ResetPin : public Module {
  private:

    volatile bool *ptrReset;     // pointer to the data source
    GpioPinRefs pin;

  public:

    ResetPin(volatile bool&, GpioPinRefs&);
    virtual void update(void);
    virtual void slowUpdate(void);

};

#endif /* MODULES_RESETPIN_H_ */
