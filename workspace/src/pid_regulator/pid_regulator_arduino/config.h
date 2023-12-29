#ifndef PID_REGULATOR_CONFIG_H
#define PID_REGULATOR_CONFIG_H


#define PWM_PIN               9

#define ENCA                  2 // YELLOW
#define ENCB                  8 // GREEN

#define F_PIN                A2
#define B_PIN                A3

#define SERIAL_BAUDRATE 2000000
#define START_BYTE           64
#define START_BYTE0_CELL      0
#define START_BYTE1_CELL      1
#define LENGTH_CELL           2

#define PING_CMD_SIZE         4
#define PING_MSG_SIZE         4

#define PID_CMD_SIZE         16

#define POSE_MSG_SIZE        31

#define KP_IDX                3
#define KD_IDX                7
#define KI_IDX               11

#define KP_SIZE               4
#define KD_SIZE               4
#define KI_SIZE               4

#define POSE_IDX              3
#define TARGET_IDX           12
#define U_IDX                21

#define POSE_SIZE             9
#define TARGET_SIZE           9
#define U_SIZE                9

#define PING_CALLBACK         0
#define PID_CALLBACK          1
#define RESET_CALLBACK        2

#define CALLBACKS_CNT         3


#endif // PID_REGULATOR_CONFIG_H
