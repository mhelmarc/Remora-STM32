#include "eStop.h"


eStop::eStop(volatile int32_t &ptrTxHeader, GpioPinRefs& pin) :
    ptrTxHeader(&ptrTxHeader),
	pin(pin)
{
  /* Pin used for this is an INPUT, mode is already set before calling this class */
}



void eStop::update()
{
    if (this->pin.read() == true)
    {
        *ptrTxHeader = PRU_ESTOP;
    }
    else {
        *ptrTxHeader = PRU_DATA;
    }
}

void eStop::slowUpdate()
{
	return;
}
