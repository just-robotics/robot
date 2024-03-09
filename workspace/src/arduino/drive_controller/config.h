#ifndef DRIVE_CONTROLLER_CONFIG_H
#define DRIVE_CONTROLLER_CONFIG_H


#define MOTOR_0_F_PIN         2
#define MOTOR_0_B_PIN         3

#define MOTOR_1_F_PIN         4
#define MOTOR_1_B_PIN         5

#define MOTOR_2_F_PIN         6
#define MOTOR_2_B_PIN         7

#define MOTOR_3_F_PIN         8
#define MOTOR_3_B_PIN         9

#define MOTOR_0_ENCB         14 // GREEN
#define MOTOR_1_ENCB         15
#define MOTOR_2_ENCB         16
#define MOTOR_3_ENCB         17

#define MOTOR_0_ENCA         18 // YELLOW
#define MOTOR_1_ENCA         19
#define MOTOR_2_ENCA         20
#define MOTOR_3_ENCA         21

#define TPR              206.26

#define SERIAL_BAUDRATE 2000000
#define START_BYTE           64

#define START_BYTE0_IDX       0
#define START_BYTE1_IDX       1
#define DATA_IDX              2

#define CMD_SIZE             32
#define MSG_SIZE             67

#define KP_IDX                3
#define KD_IDX                7
#define KI_IDX               11

#define KP_SIZE               4
#define KD_SIZE               4
#define KI_SIZE               4

#define POSE0_IDX             2
#define POSE1_IDX            10
#define POSE2_IDX            18
#define POSE3_IDX            26
#define VEL0_IDX             34
#define VEL1_IDX             38
#define VEL2_IDX             42
#define VEL3_IDX             46
#define TARGET0_IDX          50
#define TARGET1_IDX          54
#define TARGET2_IDX          58
#define TARGET3_IDX          62

#define CMD_VEL0_IDX          2
#define CMD_VEL1_IDX          6
#define CMD_VEL2_IDX         10
#define CMD_VEL3_IDX         14
#define CMD_KP_IDX           18
#define CMD_KI_IDX           22
#define CMD_KD_IDX           26
#define CMD_RESET_IDX        30

#define CMD_VEL_SIZE          4


#endif // DRIVE_CONTROLLER_CONFIG_H
