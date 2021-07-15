/*
 * This file is copied and modified from the original GpioRef.h file
 * part of the open source stm32plus library.
 * Copyright (c) 2011,2012,2013,2014 Andy Brown <www.andybrown.me.uk>
 *
 * And is part of Remora-STM32 port of Remora https://github.com/scottalford75/Remora
 * Copyright (c) 2021 Mhel Marcelo
 */

#ifndef GPIOPINREFS_H
#define GPIOPINREFS_H

#include "config/stm32plus.h"
#include "drivers/GpioPin.h"


/**
 * Simple wrapper class for a port and a pin. Useful for when a class would need to store a reference
 * to a Gpio object but cannot because the Gpio references returned by Gpio.operator[] are generally
 * not safe to hold on to.
 *
 * It's always safe to do a bitwise copy of this class
 */

class GpioPinRefs {

  protected:
    GPIO_TypeDef *_peripheralAddress;
    uint16_t _pin;

  public:
    GpioPinRefs(){};
    GpioPinRefs(const GpioPin &gpio);
    GpioPinRefs(GPIO_TypeDef *peripheralAddress, uint16_t pin);

    void set() const;
    void reset() const;
    void setState(bool state) const;
    bool read() const;

    GPIO_TypeDef* getPeripheralAddress() const;
    uint16_t getPin() const;
    uint8_t getPinIndex() const;

    GpioPinRefs& operator=(const GpioPinRefs& src);
    GpioPinRefs& operator=(const GpioPin& src);

    bool operator==(const GpioPinRefs& src) const;
    bool operator!=(const GpioPinRefs& src) const;

    GpioPin::ModeType getMode() const;
};

/**
 * Constructor
 * @param gpio The Gpio class
 */

inline GpioPinRefs::GpioPinRefs(const GpioPin &gpio) {
  _peripheralAddress = gpio.getPeripheralAddress();
  _pin = gpio.getSelectedPin();
}

/**
 * Constructor
 * @param peripheralAddress GPIO port base
 * @param pin peripheral library compatible pin number
 */

inline GpioPinRefs::GpioPinRefs(GPIO_TypeDef *peripheralAddress, uint16_t pin)
    : _peripheralAddress(peripheralAddress),
      _pin(pin) {
}

/**
 * Return the port
 * @return The GPIO_TypeDef port address
 */

inline GPIO_TypeDef* GpioPinRefs::getPeripheralAddress() const {
  return _peripheralAddress;
}

/**
 * Return the pin. This is the pin bit mask (1,2,4,8,16...)
 * @return The pin bit mask.
 */

inline uint16_t GpioPinRefs::getPin() const {
  return _pin;
}

/**
 * Set the pin to HIGH
 */

inline void GpioPinRefs::set() const {
  _peripheralAddress->BSRRL = _pin;
}

/**
 * Set the selected pin to LOW.
 */

inline void GpioPinRefs::reset() const {
  _peripheralAddress->BSRRH = _pin;
}

/**
 * Set the pin state
 */

inline void GpioPinRefs::setState(bool state) const {
  state == true ? (_peripheralAddress->BSRRL = _pin) : (_peripheralAddress->BSRRH = _pin);
}

/**
 * Read the selected pin state.
 * @return The pin state.
 */

inline bool GpioPinRefs::read() const {
  return (_peripheralAddress->IDR & _pin) != 0 ? true : false;
}

/**
 * Assignment operator from GpioPinRefs
 * @param src the object to copy from
 * @return self
 */

inline GpioPinRefs& GpioPinRefs::operator=(const GpioPinRefs &src) {
  _peripheralAddress = src.getPeripheralAddress();
  _pin = src.getPin();
  return *this;
}

/**
 * Assignment operator from Gpio
 * @param src the object to copy from
 * @return self
 */

inline GpioPinRefs& GpioPinRefs::operator=(const GpioPin& src) {
  _peripheralAddress = src.getPeripheralAddress();
  _pin = src.getSelectedPin();
  return *this;
}

/**
 * Equality comparison operator
 * @param src the object to compare to
 * @return true if equal
 */

inline bool GpioPinRefs::operator==(const GpioPinRefs &src) const {
  return _peripheralAddress == src._peripheralAddress && _pin == src._pin;
}

/**
 * Inequality comparison operator
 * @param src the object to compare to
 * @return true if not equal
 */

inline bool GpioPinRefs::operator!=(const GpioPinRefs &src) const {
  return _peripheralAddress != src._peripheralAddress || _pin != src._pin;
}

/**
 * Get the pin index (0..15)
 * @return the pin index in the port
 */

inline uint8_t GpioPinRefs::getPinIndex() const {
  return stm32plus::bithacks::firstSetBit(_pin);
}

/**
 * Get the pin mode type (input,output,analog,alternate function)
 * @return the mode type
 */

inline GpioPin::ModeType GpioPinRefs::getMode() const {
  return GpioPin::getMode(_peripheralAddress, _pin);
}
#endif
