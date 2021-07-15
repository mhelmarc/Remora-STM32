#include "switch.h"


Switch::Switch(float SP, volatile float &ptrPV, GpioPinRefs& pin, bool mode) :
	SP(SP),
	ptrPV(&ptrPV),
	mode(mode)
{
  /* This pin is an output, it's mode is already set before this module is called */
  this->PV = 0.0;
  this->pin = pin;
}


void Switch::update()
{
	bool pinState;

	pinState = this->mode;

	// update the SP
	this->PV = *(this->ptrPV);

	if (this->PV > this->SP)
	{
		this->pin.setState(pinState);
	}
	else
	{
		pinState = !pinState;
		this->pin.setState(pinState);
	}

}


void Switch::slowUpdate()
{
	return;
}
