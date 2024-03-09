#ifndef TOF_CONFIG_H
#define TOF_CONFIG_H


#define SERIAL true

#define TOF_NUM 12
#define XSHUTS_START_PIN 34
#define START_ADDR 0x34

#define SERIAL_BAUDRATE 2000000
#define START_BYTE           64
#define TIMER               100

#define START_BYTE0_IDX       0
#define START_BYTE1_IDX       1
#define DATA_IDX              2
#define DATA_SIZE             4
#define CMD_SIZE              3
#define MSG_SIZE             51


#endif // DRIVE_CONTROLLER_CONFIG_H
