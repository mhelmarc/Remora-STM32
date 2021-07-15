#ifndef BLINK_H
#define BLINK_H

#include "modules/module.h"
#include "drivers/GpioPinRefs.h"

class Blink : public Module
{

	private:
		bool 		bState;
		uint32_t 	periodCount;
		uint32_t 	blinkCount;

		GpioPinRefs blinkPin;	// class object members - Pin objects

	public:

		Blink(GpioPinRefs&, uint32_t, uint32_t);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
