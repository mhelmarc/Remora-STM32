#ifndef REMORA_H_
#define REMORA_H_

#define JOINTS              8               // Number of joints - set this the same as LinuxCNC HAL compenent. Max 8 joints
#define VARIABLES           6               // Number of command values - set this the same as the LinuxCNC HAL component
#define SPI_BUFF_SIZE       64              // Size of SPI recieve buffer - same as HAL component, 64


typedef union {
    // this allow structured access to the incoming SPI data without having to move it
    struct {
        uint8_t buffer[SPI_BUFF_SIZE];
    } rx;

    struct {
        int32_t header;
        volatile int32_t jointFreqCmd[JOINTS]; // Base thread commands ?? - basically motion
        float setPoint[VARIABLES]; // Servo thread commands ?? - temperature SP, PWM etc
        uint8_t jointEnable;
        uint8_t outputs;
        uint8_t spare0;
        uint8_t spare1;
    } data;

} rxData_t;

extern volatile rxData_t rxData;

typedef union {
    // this allow structured access to the out going SPI data without having to move it
    struct {
        uint8_t buffer[SPI_BUFF_SIZE];
    } tx;

    struct {
        int32_t header;
        int32_t jointFeedback[JOINTS];    // Base thread feedback ??
        float processVariable[VARIABLES];            // Servo thread feedback ??
        uint8_t inputs;
    } data;

} txData_t;

extern volatile txData_t txData;

#endif /* REMORA_H_ */
