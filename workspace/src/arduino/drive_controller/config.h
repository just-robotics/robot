#ifndef DRIVE_CONTROLLER_CONFIG_H
#define DRIVE_CONTROLLER_CONFIG_H


#define MOTOR_0_F_PIN         5
#define MOTOR_0_B_PIN         6

#define MOTOR_1_F_PIN         9
#define MOTOR_1_B_PIN        10

#define MOTOR_0_ENCB          4 // GREEN
#define MOTOR_1_ENCB          7

#define MOTOR_0_ENCA          2 // YELLOW
#define MOTOR_1_ENCA          3

#define TPR             329.9125

#define SERIAL_BAUDRATE 2000000
#define START_BYTE           64

#define START_BYTE0_IDX       0
#define START_BYTE1_IDX       1
#define DATA_IDX              2

#define CMD_SIZE             24
#define MSG_SIZE             35

#define KP_IDX                3
#define KD_IDX                7
#define KI_IDX               11

#define KP_SIZE               4
#define KD_SIZE               4
#define KI_SIZE               4

#define POSE0_IDX             2
#define POSE1_IDX            10
#define VEL0_IDX             18
#define VEL1_IDX             22
#define TARGET0_IDX          26
#define TARGET1_IDX          30

#define CMD_VEL0_IDX          2
#define CMD_VEL1_IDX          6
#define CMD_KP_IDX           10
#define CMD_KI_IDX           14
#define CMD_KD_IDX           18
#define CMD_RESET_IDX        22

#define CMD_VEL_SIZE          4


#endif // DRIVE_CONTROLLER_CONFIG_H
