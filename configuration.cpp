/*
 * configuration.cpp
 *
 *  This file is part of Remora-STM32 a port of Remora
 *  (https://github.com/scottalford75/Remora) a free, opensource LinuxCNC
 *  component and Programmable Realtime Unit (PRU)
 *
 *  Copyright (C) 2021
 *
 */

#include "config/stm32plus.h"
#include <iterator>
#include <vector>

#include "drivers/serialcom.h"
#include "drivers/GpioPinRefs.h"
#include "drivers/DigitalOutputPin.h"
#include "drivers/DigitalInputPin.h"

#include "configuration.h"
#include "remora.h"

#include "modules/module.h"
#include "modules/blink/blink.h"
#include "modules/resetPin/resetPin.h"
#include "modules/stepgen/stepgen.h"
#include "modules/digitalPin/digitalPin.h"
#include "modules/eStop/eStop.h"
#include "modules/switch/switch.h"

#include "thread/pruThread.h"

#include "utils/utils.h"

/* These are declared in main, but we need access to it here */
extern PRUThread<ServoThreadTimer_t> *servoThread;
extern PRUThread<BaseThreadTimer_t> *baseThread;

/* Pointers to data */
volatile int32_t* ptrTxHeader;
volatile bool*    ptrPRUreset;
volatile int32_t* ptrJointFreqCmd[JOINTS];
volatile int32_t* ptrJointFeedback[JOINTS];
volatile uint8_t* ptrJointEnable;
volatile float*   ptrSetPoint[VARIABLES];
volatile float*   ptrProcessVariable[VARIABLES];
volatile uint8_t* ptrInputs;
volatile uint8_t* ptrOutputs;


/* Forward declarations */
void createDigitalPin(char*);
void createStepGenerator(char*, uint8_t);
void createResetter(char*,volatile bool*);
void createEStopper(char*);
void createBlinker(char*);
void createEncoder(char*);
void createSwitch(char*);

/* Get the address of the port */
inline GPIO_TypeDef* getPort(const char c) {
  GPIO_TypeDef *port = NULL;
  char ch = c & 0xDF; // force upper case
  switch (ch) {
    case 'A':
      port = GPIOA;
    break;
    case 'B':
      port = GPIOB;
    break;
    case 'C':
      port = GPIOC;
    break;
    case 'D':
      port = GPIOD;
    break;
    case 'E':
      port = GPIOE;
    break;
    case 'H':
      port = GPIOH;
    break;
#if defined(STM32PLUS_F4_HAS_GPIOF_G_I)
    case 'F':
      port = GPIOF;
    break;
    case 'G':
      port = GPIOG;
    break;
    case 'I':
      port = GPIOI;
    break;
#endif
  }
  return port;
}

inline uint16_t getPin(uint8_t p) {
  return static_cast<uint16_t>(1 << p);
}


/* No Pull-up, No Pull-down, 50MHz gpio speed as default */
inline const GpioPinRefs getInputPinRef(const char port, const uint8_t pin,
    GpioPin::PullUpDownType pupd = GpioPin::PUPD_NONE,
    GpioPin::SpeedType speed = GpioPin::FAST_SPEED)  {

  GpioPinPortBase portbase(getPort(port));
  DigitalInputPin pinref(portbase, static_cast<uint16_t>(1 << pin), pupd,
      speed);
  return pinref[pin];

}


/* No Pull-up, No Pull-down, 50MHz gpio speed, Push-Pull as default */
inline const GpioPinRefs getOutputPinRef(const char port, const uint8_t pin,
    GpioPin::OutputType ppod = GpioPin::PUSH_PULL,
    GpioPin::PullUpDownType pupd = GpioPin::PUPD_NONE,
    GpioPin::SpeedType speed = GpioPin::FAST_SPEED) {

  GpioPinPortBase portbase(getPort(port));
  DigitalOutputPin pinref(portbase, static_cast<uint16_t>(1 << pin), speed,
      ppod, pupd);
  return pinref[pin];

}



//TODO Error checking.
// if a custom GUI is used to send data through serial port, checking is probably easier there
// to save space here, data with mistake could be sent through serial port via terminal
// entered manually and need to be taken care of.
/**
 * This where modules are configured according to formatted data received from serial port
 */

int setConfiguration(volatile bool *pPRUreset) {

  char data[128];
  char *ptoken;
  char *pstr;
  char *pd;

//  int threadType;
  int configType;

  xputs("\n## Entering Configuration mode\n");
  xputs("Configuration is a work in progress, error checking is needed here\n");

  if ( xgets(data, sizeof(data)) == 0)
    return -1;

  xprintf("\nData: %s\n", data);

  /* this whole process relies heavily on the wonders of pointers, not easy to debug ;-) */
  for (pstr = xstrtok(&pd, data, '#'); pd; pstr = xstrtok(&pd, 0, '#')) {

    if (pstr == 0) {
      break; // get out of this loop
    }

    ptoken = pstr;

    switch (*ptoken++) {
      case 'S':
        xputs("\nServoThread\n");

        /* Configuration type 0...n */
        if (isNumber(*ptoken)) {
          configType = *ptoken - '0'; // convert to decimal value
//          xprintf("Type%d ", configType);
          ptoken++; // next character

          switch (configType) {
            case ESTOP:
              xputs("eStop Pin\n");
              createEStopper(ptoken);
            break;

            case RESET_PIN:
              xputs("Reset Pin\n");
              createResetter(ptoken, pPRUreset);
            break;

            case DIGITAL_PIN:
              xputs("Digital Pin\n");
              createDigitalPin(ptoken);
            break;

            case BLINKER:
              xputs("Blinker\n");
              createBlinker(ptoken);
            break;

            case SWITCH:
              xputs("Fan switch\n");
              createSwitch(ptoken);
            break;
          } //switch configtype
        }
      break;

      case 'B':
        xputs("\nBaseThread\n");

        /* Configuration type 0...n */
        if (isNumber(*ptoken)) {
          configType = *ptoken - '0'; // convert to decimal value
//          xprintf("Type%d ", configType);
          ptoken++; // next character

          switch (configType) {
            case STEPGEN:
              xputs("Step Generator\n");
              if (isNumber(*ptoken)) {
                uint8_t joint_counts = *ptoken - '0';
                ptoken++; // need to increment pointer to skip separator
                createStepGenerator(ptoken, joint_counts);
              }

            break;

            case ENCODER:
              xputs("Encoder\n");
              createEncoder(ptoken);
            break;

            case RC_SERVO:
              xputs("RC Servo\n");
            break;

          }
        }
      break;

      default:

      break;
    }
  }

  xputs("Done!\n");

  return 0;
}


/** Stepgen data format "#B14,B15D3C12,A4F10E3,C14E31G11,H35K28J11"
  * B14 B=BaseThread 1=Type1 4=Number of joints
  * B15 PortB StepPin 15
  * D3  PortD DirPin 3
  * C12 PortC EnablePin 12
  */

/* Error checking for available pins should be checked before calling this
 * I had hard time debugging this, 'till I realized I was using pin number
 * more than the available 0 to 15.
 */

void createStepGenerator(char* data, uint8_t numJoints){
  char* ptr;

  std::vector<Joints_t> joints(numJoints); // regular array won't work with variable size

  GpioPinRefs stepBit;
  GpioPinRefs directionBit;
  GpioPinRefs enableBit;

  volatile int j = 0;

  /* Get the values for every joint */
  while (j < numJoints) {
    xstrtok(&ptr, data, ',');
    getValues(ptr, joints[j].param, 3);
    xstrtok(&ptr, 0, ',');
    j++;
  }


  for (j = 0; j < numJoints; j++) {

    xprintf("Joint %d\n", j);

    xprintf(" Step pin at Port%c pin %d\n", joints[j].param[0].key,
        joints[j].param[0].value);

    xprintf(" Direction pin at Port%c pin %d\n", joints[j].param[1].key,
        joints[j].param[1].value);

    xprintf(" Enable pin at Port%c pin %d\n", joints[j].param[2].key,
        joints[j].param[2].value);

    stepBit = getOutputPinRef(joints[j].param[0].key, joints[j].param[0].value);
    directionBit = getOutputPinRef(joints[j].param[1].key, joints[j].param[1].value);
    enableBit = getOutputPinRef(joints[j].param[2].key, joints[j].param[2].value);

    // configure pointers to data source and feedback location
    ptrJointFreqCmd[j] = &rxData.data.jointFreqCmd[j];
    ptrJointFeedback[j] = &txData.data.jointFeedback[j];
    ptrJointEnable = &rxData.data.jointEnable; //FIXME this should be outside the loop

    Module *stepgen = new Stepgen(static_cast<int32_t>(PRU_BASEFREQ), j,
        enableBit, stepBit, directionBit, STEPBIT, *ptrJointFreqCmd[j],
        *ptrJointFeedback[j], *ptrJointEnable);

    baseThread->registerModule(stepgen);

  }

}



/** Blinker data format "#S8,E0F4"
 *  S8 = ServoThread Type8
 *  E0 = PortE Pin 0
 *  F4 = Frequency of the blink, only value is used
 *       blink speed is calculated  with (PRU_SERVOFREQ / F4) / 2
 */
void createBlinker(char *data) {
  char *ptr;
  KeyPair_t keypairs[2];
  uint32_t freq;

  xstrtok(&ptr, data, ',');
  getValues(ptr, keypairs, 2);

  xprintf("Make Blink at Port%c pin %d\n", keypairs[0].key,
      keypairs[0].value);

  GpioPinRefs pinbit = getOutputPinRef(keypairs[0].key,
      keypairs[0].value);

  freq = static_cast<uint32_t>(keypairs[1].value);

  Module *blinker = new Blink(pinbit, PRU_SERVOFREQ, freq);
  servoThread->registerModule(blinker);
}

/** Switch data format "#S7,E0F4"
 *  S7 = ServoThread Type8
 *  A5 = PortA Pin 5
 *  M1 = Mode 0=off 1=on only value is used
 *  V0 = PV index only value is used
 *  S25.5 = Set point only value is used
 */
void createSwitch(char* data) {
  char *ptr;
  float sp;
  int pv;
  int mode;
  KeyPair_t keypairs[4];

  xstrtok(&ptr, data, ',');
  getValues(ptr, keypairs, 4);

  mode = keypairs[1].value;
  const char *modestring = mode ? "On" : "Off";

  xprintf("Make Switch (%s) at Port%c pin %d\n", modestring, keypairs[0].key,
      keypairs[0].value);

  GpioPinRefs pinbit = getOutputPinRef(keypairs[0].key,
      keypairs[0].value);

  pv = keypairs[2].value;
  sp = keypairs[3].fvalue;

  if (mode) {
    Module* SoftSwitch = new Switch(sp, *ptrProcessVariable[pv], pinbit, 1);
    servoThread->registerModule(SoftSwitch);
  }
  else {
    Module* SoftSwitch = new Switch(sp, *ptrProcessVariable[pv], pinbit, 0);
    servoThread->registerModule(SoftSwitch);
  }

}

/** Digital Pin data format "#S2,A6M1B0I0P0U1Z0"
 *  S2 = ServoThread Type 2
 *  A6 = PortA Pin 6
 *  M1 = Mode 1=input 0=output, only value is used
 *  B0 = Data bit, only value is used
 *  I0 = Invert 1=inverted 0=norma, only value is used
 *  P0 = Output type 0=PUSH_PULL 1=OPEN_DRAIN
 *  U1 = PullUp/Down 0=none 1=pull-up 2=pull-down
 *  Z2 = Speed 0=low 1=medium 2=fast 3=high
 **/
void createDigitalPin(char *data) {
  char* ptr;
  KeyPair_t keypairs[7];

  GpioPin::OutputType ppod;
  GpioPin::PullUpDownType pupd;
  GpioPin::SpeedType speed;
  GpioPinRefs pinbit;

  xstrtok(&ptr, data, ',');
  getValues(ptr, keypairs, 7);

  const char *mode =(keypairs[DigiPin::MODE].value == 0) ? "Output" : "Input";

  ptrOutputs = &rxData.data.outputs;
  ptrInputs = &txData.data.inputs;

  ppod = static_cast<GpioPin::OutputType>(keypairs[DigiPin::PPOD].value);
  pupd = static_cast<GpioPin::PullUpDownType>(keypairs[DigiPin::PUPD].value);
  speed = static_cast<GpioPin::SpeedType>(keypairs[DigiPin::SPEED].value);

  xprintf("Make Digital %s at Port%c pin %d", mode, keypairs[DigiPin::PORT].key,
      keypairs[DigiPin::PORT].value);

  /* it's an output */
  if (keypairs[DigiPin::MODE].value == 0) {
    pinbit = getOutputPinRef(keypairs[0].key, keypairs[0].value, ppod, pupd,
        speed);

    Module *digitalPin = new DigitalPin(*ptrOutputs,
        keypairs[DigiPin::MODE].value, pinbit,
        keypairs[DigiPin::DATA_BIT].value, keypairs[DigiPin::INVERT].value);

    servoThread->registerModule(digitalPin);
  }
  /* it's an input */
  else if (keypairs[DigiPin::MODE].value == 1) {
    pinbit = getInputPinRef(keypairs[DigiPin::PORT].key,
        keypairs[DigiPin::PORT].value, pupd, speed);

    Module *digitalPin = new DigitalPin(*ptrInputs,
        keypairs[DigiPin::MODE].value, pinbit,
        keypairs[DigiPin::DATA_BIT].value, keypairs[DigiPin::INVERT].value);

    servoThread->registerModule(digitalPin);
  }
}



/** Reset Pin data format "#S3,C15;"
 *  S3 ServoThread Type3
 *  C15 PortC Pin15
 */
void createResetter(char* data, volatile bool* prureset) {
  char* ptr;
  KeyPair_t keypairs;

  xstrtok(&ptr, data, ',');
  getValues(ptr, &keypairs, 1);

  ptrPRUreset = prureset;

  xprintf("Make Reset at Port%c pin %d\n", keypairs.key, keypairs.value);

  GpioPinRefs pinbit = getInputPinRef(keypairs.key, keypairs.value);

  Module *resetter;
  resetter = new ResetPin(*ptrPRUreset, pinbit);
  servoThread->registerModule(resetter);
}


/** eStop Pin data format "#S0,C12"
 *  S0 ServoThread Type 0
 *  C12 PortC Pin12
 */
void createEStopper(char* data) {
  char* ptr;
  KeyPair_t keypairs;

  xstrtok(&ptr, data, ',');
  getValues(ptr, &keypairs, 1);

  ptrTxHeader = &txData.data.header;

  xprintf("Make eStop at Port%c pin %d\n", keypairs.key, keypairs.value);

  GpioPinRefs pinbit = getInputPinRef(keypairs.key, keypairs.value);

  Module *estop = new eStop(*ptrTxHeader, pinbit);
  servoThread->registerModule(estop);
}


/** Encoder data format "#B6,A8B4I7V2Q0"
 *  B6 BaseThread Type 6
 *  A8 Channel A PortA Pin 8
 *  B4 Channel B PortB Pin 4
 *  I7 Index Channel PortI Pin 7
 *  V2 PV index Only value is used
 *  Q0 Modifier 0=none 1=OpenDrain 2=PullUp 3=PullDown 4=PullNone
 *     only value is used
 */
void createEncoder(char* data) {
  char* ptr;
  KeyPair_t keypairs[5];
  uint8_t count = 0;

  xstrtok(&ptr, data, ',');
  getValues(ptr, keypairs, 5);

  /* NOT AVAILABLE YET */
  while (count < 5) {
    xprintf(" EncParam %c Value %d\n", keypairs[count].key,
        keypairs[count].value);
    count++;
  }
}

