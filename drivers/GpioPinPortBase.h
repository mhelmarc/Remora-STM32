/*
 * This file is copied and modified from the original GpioPortBase.h file
 * part of the open source stm32plus library.
 * Copyright (c) 2011,2012,2013,2014 Andy Brown <www.andybrown.me.uk>
 *
 * And is part of Remora-STM32 port of Remora https://github.com/scottalford75/Remora
 *
 * Copyright (c) 2021
 *
 */

  class GpioPin;

  /**
   * Base class for the GpioPort template class
   */

  class GpioPinPortBase {

    public:
      GpioPin *_pinHandlers[16];
      GPIO_TypeDef *_peripheralAddress;
      uint8_t _low;

    public:
      GpioPinPortBase(GPIO_TypeDef *peripheralAddress);

      void setPinHandler(uint8_t index,GpioPin *pinHandler);
      GPIO_TypeDef *getPeripheralAddress() const;
  };


  /**
   * Constructor
   * @param peripheralAddress
   */

  inline GpioPinPortBase::GpioPinPortBase(GPIO_TypeDef *peripheralAddress)
    : _peripheralAddress(peripheralAddress) {
    memset(_pinHandlers,'\0',sizeof(_pinHandlers));
    _low=15;
  }


  /**
   * Set the pin handler for the pin at a given position
   * @param index The position (0..15)
   * @param pinHandler The handler for that position
   */

  inline void GpioPinPortBase::setPinHandler(uint8_t index,GpioPin *pinHandler) {

    _pinHandlers[index]=pinHandler;

    if(index<_low)
      _low=index;
  }


  /**
   * Cast to the GPIO peripheral address
   */

  inline GPIO_TypeDef *GpioPinPortBase::getPeripheralAddress() const {
    return _peripheralAddress;
  }
