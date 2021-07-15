#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#define PRU_BASEFREQ    	40000 //24000   // PRU Base thread ISR update frequency (hz)
#define PRU_SERVOFREQ       1000            // PRU Servo thread ISR update freqency (hz)
#define STEPBIT     		22            	// bit location in DDS accum
#define STEP_MASK   		  (1L<<STEPBIT)

#define JSON_BUFF_SIZE	    10000			// Jason dynamic buffer size

//#define JOINTS			    8				// Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
//#define VARIABLES           6             	// Number of command values - set this the same as the LinuxCNC HAL compenent

#define PRU_DATA		    0x64617461 	    // "data" SPI payload
#define PRU_READ            0x72656164      // "read" SPI payload
#define PRU_WRITE           0x77726974      // "writ" SPI payload
#define PRU_ESTOP           0x65737470      // "estp" SPI payload


// Serial configuration
/*
  USART1 on PortA
  TX    PA9
  RX    PA10
 */
#define PC_BAUD             115200          // UART baudrate


#define LOOP_TIME           100             // msec
#define SPI_ERR_MAX         5
// PRU reset will occur in SPI_ERR_MAX * LOOP_TIME = 0.5sec

/*
 SPI1 on Port A4:A7 on STM32F4
 NSS    PA4
 SCK    PA5
 MiSO   PA6
 MOSI   PA7
*/


enum TypeOfFunction {
  ESTOP = 0,
  STEPGEN,
  DIGITAL_PIN,
  RESET_PIN,
  RC_SERVO,
  TEMPERATURE,
  ENCODER,
  SWITCH,
  BLINKER
};



/*
 * DigitalPin properties"
 */
enum  DigiPin {
    PORT = 0,
    MODE,
    DATA_BIT,
    INVERT,
    PPOD,
    PUPD,
    SPEED
};



typedef struct {
    char key;
    union {
        uint8_t value;
        float fvalue;
    };
} KeyPair_t;



typedef struct  {
    KeyPair_t param[3];
}Joints_t;


int setConfiguration(volatile bool*);

#endif
