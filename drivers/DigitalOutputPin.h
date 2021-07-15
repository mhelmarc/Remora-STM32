/*
 * This file is copied and modified from the original DigitalOutputFeature.h file
 * part of the open source stm32plus library.
 * Copyright (c) 2011,2012,2013,2014 Andy Brown <www.andybrown.me.uk>
 *
 * And is part of Remora-STM32 port of Remora https://github.com/scottalford75/Remora
 * Copyright (c) 2021
 */

#ifndef DIGITAL_OUTPUT_PIN_H
#define DIGITAL_OUTPUT_PIN_H

#include "GpioPin.h"

class DigitalOutputPin: public GpioPin {

  public:

    /**
     * Constructor
     * @param port the port reference
     */

    DigitalOutputPin(GpioPinPortBase &port, uint16_t pinIds,
            GpioPin::SpeedType speed = GpioPin::FAST_SPEED, GpioPin::OutputType outputType =
            GpioPin::PUSH_PULL, GpioPin::PullUpDownType pullUpDownType=GpioPin::PUPD_NONE) :
        GpioPin(port){

      enableClock();
      initialise(speed, outputType, pullUpDownType, pinIds);

    }

    /**
     * Array operator. Return a reference to the selected pin. It is not generally safe to store the
     * reference returned by this function between calls to this index operator because subsequent
     * calls can change the pin used by that reference.
     * @param selectedPin The pin to select
     * @return A reference to the pin that you can call operations such as set(), reset() on.
     */

    GpioPin& operator[](uint8_t selectedPin) {
      _portBase._pinHandlers[selectedPin]->setSelectedPin(selectedPin);
      return *(_portBase._pinHandlers[selectedPin]);
    }

    /**
     * @param speed
     * @param PinOutputType
     * @param PinPullUpDownType
     * @param pinIds
     */

    void initialise(GpioPin::SpeedType speed, GpioPin::OutputType outputType,
        GpioPin::PullUpDownType pullUpDownType, uint16_t pinIds) {

      uint8_t i;
      uint8_t pinpos;

      (void) pullUpDownType;

      this->_pinIds = pinIds;

      for (pinpos = 0; pinpos < 0x10; pinpos++) {

        if ((_pinIds & (1 << pinpos)) == 0) {
          continue;
        }

        _peripheralAddress->OSPEEDR = (_peripheralAddress->OSPEEDR
            & ~(static_cast<uint32_t>(0x03 << (2 * pinpos))))
            | (static_cast<uint32_t>(speed << (2 * pinpos)));

        _peripheralAddress->MODER = (_peripheralAddress->MODER
            & ~(static_cast<uint32_t>(0x03 << (2 * pinpos))))
            | (static_cast<uint32_t>(GpioPin::OUTPUT << (2 * pinpos)));

        _peripheralAddress->OTYPER = (_peripheralAddress->OTYPER
            & ~static_cast<uint16_t>(0x01 << pinpos))
            | (static_cast<uint16_t>(outputType << pinpos));

        _peripheralAddress->PUPDR = (_peripheralAddress->PUPDR & ~(0x03 << (2 * pinpos)))
            | (static_cast<uint32_t>(pullUpDownType << (2 * pinpos)));

      }

      // set ourselves as the pin handler in the port base

      for (i = 0; i < 16; i++)
        if ((pinIds & (1 << i)) != 0)
          _portBase.setPinHandler(i, this);
    }
};

#endif
