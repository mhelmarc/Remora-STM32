/*
 * This file is copied and modified from the original Gpio.h file
 * part of the open source stm32plus library.
 * Copyright (c) 2011,2012,2013,2014 Andy Brown <www.andybrown.me.uk>
 * With some parts taken from GpiO library by Tilen Majerle https://stm32f4-discovery.net/
 *
 * And is part of Remora-STM32 port of Remora https://github.com/scottalford75/Remora
 *
 * Copyright (c) 2021
 */

#ifndef GPIO_PIN_H_
#define GPIO_PIN_H_
// ensure the MCU series is correct
#ifndef STM32PLUS_F4
#warning This class can only be used with the STM32F4 series
#endif

#include "util/BitHacks.h"
#include "GpioPinPortBase.h"
  /**
   * @brief Base class for holding common pin functionality. Inherits from the
   * GPIO_InitTypeDef structure
   */

  class GpioPin {

    protected:
      GpioPinPortBase& _portBase;
      GPIO_TypeDef *_peripheralAddress;

      uint16_t _pinIds{0};
      uint16_t _selectedPin{0};

    public:
      enum ModeType {
        INPUT = 0x00,
        OUTPUT = 0x01,
        ALTERNATE_FUNCTION = 0x02,
        ANALOG = 0x03
      };

      enum OutputType {
        PUSH_PULL  = 0x00,
        OPEN_DRAIN = 0x01

      };

      enum PullUpDownType {
        PUPD_NONE =0x00,
        PUPD_UP = 0x01,  //!< PP_UP
        PUPD_DOWN = 0x02 //!< PP_DOWN
      };

      enum SpeedType {
        LOW_SPEED = 0x00,       // 2MHz
        MEDIUM_SPEED = 0x01,    // 25MHz
        FAST_SPEED = 0x02,      // 50MHz
        HIGH_SPEED = 0x03       // 100MHz
      };

    protected:
      void enableClock();
      void disableClock();
      void lockRegister();
      void getPortAndPinSource(uint8_t& portSource,uint8_t& pinSource) const;
      uint16_t getPortSource();

    public:
      GpioPin(GpioPinPortBase& portBase);

      void set() const;
      void reset() const;
      void setAll() const;
      void resetAll() const;
      void setState(bool state) const;
      void setStateAll(bool state) const;
      bool read() const;
      void enableExti() const;

      volatile uint16_t *getSetRegister() const;
      volatile uint16_t *getResetRegister() const;
      volatile uint32_t *getOutputRegister() const;

      void initialiseAF(uint16_t pinIds,
                        uint8_t afSelection,
                        GPIOSpeed_TypeDef speed=GPIO_Speed_50MHz,
                        GpioPin::OutputType outputType=GpioPin::PUSH_PULL,
                        GpioPin::PullUpDownType pullUpDownType=GpioPin::PUPD_NONE);

      void setSelectedPin(uint8_t pinNumber);
      uint16_t getSelectedPin() const;

      GPIO_TypeDef *getPeripheralAddress() const;

      static ModeType getMode(GPIO_TypeDef *peripheralAddress,uint16_t pin);
      ModeType getMode() const;
  };

  /**
   * Constructor
   * @param port
   */

  inline GpioPin::GpioPin(GpioPinPortBase& portBase)
    : _portBase(portBase),
      _peripheralAddress(portBase.getPeripheralAddress()) {
  }


  /**
   * Get the currently selected pin id
   * @return The pin ID
   */

  inline uint16_t GpioPin::getSelectedPin() const {
    return _selectedPin;
  }


  /**
   * Cast to the port typedef
   */

  inline GPIO_TypeDef *GpioPin::getPeripheralAddress() const {
    return _peripheralAddress;
  }


  /**
   * Set the selected pin number to one of those that you initialised this class with
   * @param pinNumber The pin number (0..15)
   */

  inline void GpioPin::setSelectedPin(uint8_t pinNumber) {
    _selectedPin=1 << pinNumber;
  }


  /**
   * Set the selected pin to HIGH.
   */

  inline void GpioPin::set() const {
    _peripheralAddress->BSRRL = _selectedPin;
  }


  /**
   * Set all pins managed by this class to HIGH
   */

  inline void GpioPin::setAll() const {
    _peripheralAddress->BSRRL = _pinIds;
  }


  /**
   * Set the selected pin to LOW.
   */

  inline void GpioPin::reset() const {
    _peripheralAddress->BSRRH = _selectedPin;
  }


  /**
   * Set all the pins managed by this class to LOW.
   */

  inline void GpioPin::resetAll() const {
    _peripheralAddress->BSRRH = _pinIds;
  }


  /**
   * Allow setting/resetting of the selected pin from a variable.
   * @param[in] state The new state of the pin.
   */

  inline void GpioPin::setState(bool state) const {
    if (state)
      _peripheralAddress->BSRRL = _selectedPin;
    else
      _peripheralAddress->BSRRH = _selectedPin;
  }


  /**
   * Allow setting/resetting of all pins managed by this class from a variable.
   * @param[in] state The new state of the pin.
   */

  inline void GpioPin::setStateAll(bool state) const {
    if (state)
      _peripheralAddress->BSRRL = _pinIds;
    else
      _peripheralAddress->BSRRH = _pinIds;
  }


  /**
   * Read the selected pin state.
   * @return The pin state.
   */

  inline bool GpioPin::read() const {
    return (_peripheralAddress->IDR & _selectedPin) == 0 ? false : true;
  }


  /**
   * Enable EXTI for this port/selected pin
   */

  inline void GpioPin::enableExti() const {

    uint8_t portSource,pinSource;

    getPortAndPinSource(portSource,pinSource);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);
    SYSCFG_EXTILineConfig(portSource,pinSource);
  }


  /**
   * Get the port and pin source for EXTI
   * @param[out] portSource The port source
   * @param[out] pinSource The pin source
   */

  inline void GpioPin::getPortAndPinSource(uint8_t& portSource,uint8_t& pinSource) const {

    switch(reinterpret_cast<uint32_t>(_peripheralAddress)) {
      case GPIOA_BASE: portSource=EXTI_PortSourceGPIOA; break;
      case GPIOB_BASE: portSource=EXTI_PortSourceGPIOB; break;
      case GPIOC_BASE: portSource=EXTI_PortSourceGPIOC; break;
      case GPIOD_BASE: portSource=EXTI_PortSourceGPIOD; break;
      case GPIOE_BASE: portSource=EXTI_PortSourceGPIOE; break;
      case GPIOH_BASE: portSource=EXTI_PortSourceGPIOH; break;

#if defined(STM32PLUS_F4_HAS_GPIOF_G_I)
      case GPIOF_BASE: portSource=EXTI_PortSourceGPIOF; break;
      case GPIOG_BASE: portSource=EXTI_PortSourceGPIOG; break;
      case GPIOI_BASE: portSource=EXTI_PortSourceGPIOI; break;
#endif

#if defined(STM32PLUS_F4_HAS_GPIOJ_K)
      case GPIOJ_BASE: portSource=EXTI_PortSourceGPIOJ; break;
      case GPIOK_BASE: portSource=EXTI_PortSourceGPIOK; break;
#endif
      default: portSource=0; break;
    }

    switch(_selectedPin) {
      case GPIO_Pin_0: pinSource=EXTI_PinSource0; break;
      case GPIO_Pin_1: pinSource=EXTI_PinSource1; break;
      case GPIO_Pin_2: pinSource=EXTI_PinSource2; break;
      case GPIO_Pin_3: pinSource=EXTI_PinSource3; break;
      case GPIO_Pin_4: pinSource=EXTI_PinSource4; break;
      case GPIO_Pin_5: pinSource=EXTI_PinSource5; break;
      case GPIO_Pin_6: pinSource=EXTI_PinSource6; break;
      case GPIO_Pin_7: pinSource=EXTI_PinSource7; break;
      case GPIO_Pin_8: pinSource=EXTI_PinSource8; break;
      case GPIO_Pin_9: pinSource=EXTI_PinSource9; break;
      case GPIO_Pin_10: pinSource=EXTI_PinSource10; break;
      case GPIO_Pin_11: pinSource=EXTI_PinSource11; break;
      case GPIO_Pin_12: pinSource=EXTI_PinSource12; break;
      case GPIO_Pin_13: pinSource=EXTI_PinSource13; break;
      case GPIO_Pin_14: pinSource=EXTI_PinSource14; break;
      case GPIO_Pin_15: pinSource=EXTI_PinSource15; break;
      default: pinSource=0; break;
    }
  }


  /**
   * Get the register for setting bits.
   * @return The register address
   */

  inline volatile uint16_t *GpioPin::getSetRegister() const {
    return reinterpret_cast<volatile uint16_t *>(&_peripheralAddress->BSRRL);
  }


  /**
   * Get the register for clearing bits.
   * @return The register address
   */

  inline volatile uint16_t *GpioPin::getResetRegister() const {
    return reinterpret_cast<volatile uint16_t *>(&_peripheralAddress->BSRRH);
  }


  /**
   * Get the register for writing data. As per the docs only 32-bit
   * access is permitted.
   * @return
   */

  inline volatile uint32_t *GpioPin::getOutputRegister() const {
    return reinterpret_cast<volatile uint32_t *>(&_peripheralAddress->ODR);
  }


  /**
   * Alternative to template initialisation - initialise programatically
   * @param speed
   * @param TOutputType
   * @param TPullUpDownType
   * @param pinIds
   * @param afSelection
   */

  inline void GpioPin::initialiseAF(uint16_t pinIds,
                                 uint8_t afSelection,
                                 GPIOSpeed_TypeDef speed,
                                 GpioPin::OutputType outputType,
                                 GpioPin::PullUpDownType pullUpDownType) {
    uint8_t source;
    uint32_t bit;
    GPIO_InitTypeDef init;

    (void)pullUpDownType;     // unused

    _pinIds=pinIds;

    init.GPIO_Speed=speed;
    init.GPIO_Mode=GPIO_Mode_AF;
    init.GPIO_OType=outputType==GpioPin::PUSH_PULL ? GPIO_OType_PP : GPIO_OType_OD;
    init.GPIO_PuPd=pullUpDownType==PUPD_NONE ? GPIO_PuPd_NOPULL : (pullUpDownType==PUPD_UP ? GPIO_PuPd_UP : GPIO_PuPd_DOWN);
    init.GPIO_Pin=pinIds;

    GPIO_Init(_peripheralAddress,&init);

    // need to configure each of the selected pins for AF

    for(bit=1,source=0;bit<=0x8000;bit<<=1,source++) {
      if((pinIds & bit)!=0) {
        GPIO_PinAFConfig(_peripheralAddress,source,afSelection);
        _portBase.setPinHandler(source,this);
      }
    }
  }


  /**
   * Get the pin mode type (input,output,analog,alternate function)
   * @param peripheralAddress the peripheral register address
   * @param pin the pin bitmask
   * @return The mode type
   */

  inline GpioPin::ModeType GpioPin::getMode(GPIO_TypeDef *peripheralAddress,
      uint16_t pin) {

    uint8_t pinIndex;

    pinIndex = stm32plus::bithacks::firstSetBit(pin);

    switch ((peripheralAddress->MODER >> (pinIndex * 2)) & 0x3) {

      case 0:
        return GpioPin::INPUT;

      case 1:
        return GpioPin::OUTPUT;

      case 2:
        return GpioPin::ALTERNATE_FUNCTION;

      case 3:
      default:
        return GpioPin::ANALOG;
    }
  }


  /**
   * Get the pin mode type (input,output,analog,alternate function)
   * @return the mode type
   */

  inline GpioPin::ModeType GpioPin::getMode() const {
    return getMode(_peripheralAddress,_selectedPin);
  }


  inline uint16_t GpioPin::getPortSource() {
    /* Get port source number */
    /* Offset from GPIOA                       Difference between 2 GPIO addresses */
    return (reinterpret_cast<uint32_t>(_peripheralAddress) - (GPIOA_BASE))
        / ((GPIOB_BASE) - (GPIOA_BASE));
  }


  inline void GpioPin::disableClock() {
    RCC->AHB1ENR &= ~( 1 << getPortSource() );
  }


  inline void GpioPin::enableClock() {
    RCC->AHB1ENR |= ( 1 << getPortSource() );
  }

  inline void GpioPin::lockRegister() {
    uint32_t d;

    /* Set GPIO pin with 16th bit set to 1 */
    d = 0x00010000 | _selectedPin;
    /* Write to LCKR register */
    _peripheralAddress->LCKR = d;
    _peripheralAddress->LCKR = _selectedPin;
    _peripheralAddress->LCKR = d;

    /* Read twice */
    (void) _peripheralAddress->LCKR;
    (void) _peripheralAddress->LCKR;
  }
#endif /* GPIO_PIN_H_ */
