#ifndef SWITCH_H
#define SWITCH_H

#include "modules/module.h"
#include "drivers/GpioPinRefs.h"


class Switch : public Module
{

	private:

		volatile float* ptrPV; 			// pointer to the data source
		float 			PV;
		float 			SP;
		bool			mode;			// 0 switch off, 1 switch on

		GpioPinRefs     pin;


	public:

		Switch(float, volatile float&, GpioPinRefs&, bool);

		virtual void update(void);
		virtual void slowUpdate(void);
};

#endif
