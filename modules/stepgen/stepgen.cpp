#include "stepgen.h"


/***********************************************************************
*                METHOD DEFINITIONS                                    *
************************************************************************/


Stepgen::Stepgen(int32_t threadFreq, int jointNumber,  GpioPinRefs& epin,
     GpioPinRefs& spin,  GpioPinRefs& dpin, int stepBit,
    volatile int32_t &ptrFrequencyCommand, volatile int32_t &ptrFeedback,
    volatile uint8_t &ptrJointEnable) :
    jointNumber(jointNumber),enable(epin), step(spin), direction(dpin), stepBit(
        stepBit), ptrFrequencyCommand(&ptrFrequencyCommand), ptrFeedback(
        &ptrFeedback), ptrJointEnable(&ptrJointEnable) {

  this->rawCount = 0;
  this->DDSaddValue = 0;
  this->DDSaccumulator = 0;

//  this->enable = epin;
//  this->step = spin;
//  this->direction = dpin;

  this->DDSaccumulator = 0;
  this->frequencyScale = (float) (1 << this->stepBit) / (float) threadFreq;
  this->mask = 1 << this->jointNumber;
  this->isEnabled = false;
  this->isForward = false;

}


void Stepgen::update()
{
	// Use the standard Module interface to run makePulses()
	this->makePulses();
}

void Stepgen::slowUpdate()
{
	return;
}

void Stepgen::makePulses()
{
	int32_t stepNow = 0;

	this->isEnabled = ((*(this->ptrJointEnable) & this->mask) != 0);

	if (this->isEnabled == true)  												// this Step generator is enables so make the pulses
	{
		this->enable.setState(false);                                			// Enable the driver - CHANGE THIS TO MAKE THE OUTPUT VALUE CONFIGURABLE???

		this->frequencyCommand = *(this->ptrFrequencyCommand);            		// Get the latest frequency command via pointer to the data source
		this->DDSaddValue = this->frequencyCommand * this->frequencyScale;		// Scale the frequency command to get the DDS add value
		stepNow = this->DDSaccumulator;                           				// Save the current DDS accumulator value
		this->DDSaccumulator += this->DDSaddValue;           	  				// Update the DDS accumulator with the new add value
		stepNow ^= this->DDSaccumulator;                          				// Test for changes in the low half of the DDS accumulator
		stepNow &= (1L << this->stepBit);                         				// Check for the step bit
		this->rawCount = this->DDSaccumulator >> this->stepBit;   				// Update the position raw count

		if (this->DDSaddValue > 0)												// The sign of the DDS add value indicates the desired direction
		{
			this->isForward = true;
		}
		else //if (this->DDSaddValue < 0)
		{
			this->isForward = false;
		}

		if (stepNow)
		{
			this->direction.setState(this->isForward);      // Set direction pin
			this->step.setState(true);						// Raise step pin - A4988 / DRV8825 stepper drivers only need 200ns setup time
			*(this->ptrFeedback) = this->DDSaccumulator;    // Update position feedback via pointer to the data receiver
		}
		else
		{
			this->step.setState(false);						// Reset step pin
		}

	}
	else
	{
		this->enable.setState(true);
	}

}

void Stepgen::setEnabled(bool state)
{
	this->isEnabled = state;
}
