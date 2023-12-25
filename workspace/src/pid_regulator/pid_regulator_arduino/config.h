#ifndef PID_REGULATOR_CONFIG_H
#define PID_REGULATOR_CONFIG_H


#define ENCA                  2 // YELLOW
#define ENCB                  3 // GREEN

#define F_PIN                10
#define B_PIN                11

#define BUTTON_PIN            7

#define TARGET              200

#define SERIAL_BAUDRATE 2000000
#define START_BYTE           64
#define START_BYTE0_CELL      0
#define START_BYTE1_CELL      1

#define PING_CMD_SIZE         3
#define PING_MSG_SIZE         3

#define PID_CMD_SIZE         15

#define POSE_MSG_SIZE        30

#define KP_IDX                2
#define KD_IDX                6
#define KI_IDX               10

#define KP_SIZE               4
#define KD_SIZE               4
#define KI_SIZE               4

#define POSE_IDX              2
#define TARGET_IDX           11
#define U_IDX                20

#define POSE_SIZE             9
#define TARGET_SIZE           9
#define U_SIZE                9


#endif // PID_REGULATOR_CONFIG_H
