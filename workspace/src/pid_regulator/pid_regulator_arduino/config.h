#ifndef PID_REGULATOR_CONFIG_H
#define PID_REGULATOR_CONFIG_H


#define MOTOR_0_PWM_PIN       5
#define MOTOR_1_PWM_PIN       6
#define MOTOR_2_PWM_PIN       9
#define MOTOR_3_PWM_PIN      10

// MEGA
#define MOTOR_0_ENCA         18 // YELLOW
#define MOTOR_1_ENCA         19
#define MOTOR_2_ENCA         20
#define MOTOR_3_ENCA         21

#define MOTOR_0_ENCB         14 // GREEN
#define MOTOR_1_ENCB         15
#define MOTOR_2_ENCB         16
#define MOTOR_3_ENCB         17
/*
// UNO
#define MOTOR_0_ENCA         2 // YELLOW
#define MOTOR_1_ENCA         2
#define MOTOR_2_ENCA         2
#define MOTOR_3_ENCA         2

#define MOTOR_0_ENCB         2 // GREEN
#define MOTOR_1_ENCB         2
#define MOTOR_2_ENCB         2
#define MOTOR_3_ENCB         2
*/
#define MOTOR_0_F_PIN        12
#define MOTOR_0_B_PIN        13

#define MOTOR_1_F_PIN        A0
#define MOTOR_1_B_PIN        A1

#define MOTOR_2_F_PIN        A2
#define MOTOR_2_B_PIN        A3

#define MOTOR_3_F_PIN        A4
#define MOTOR_3_B_PIN        A5

#define KP                  1.0
#define KI                  0.0
#define KD                  0.0

#define SERIAL_BAUDRATE 2000000
#define START_BYTE           64

#define START_BYTE0_IDX       0
#define START_BYTE1_IDX       1
#define TASK_IDX              2

#define PING_TASK             0
#define DISCONNECT_TASK       1
#define PID_TASK              2
#define RESET_PID_TASK        3
#define OK_TASK               4
#define POSE_TASK             5

#define TASK_CNT              6

#define PING_SIZE             4
#define DISCONNECT_SIZE       4
#define PID_CMD_SIZE         16
#define RESET_PID_CMD_SIZE    4

#define POSE_MSG_SIZE        40

#define KP_IDX                3
#define KD_IDX                7
#define KI_IDX               11

#define KP_SIZE               4
#define KD_SIZE               4
#define KI_SIZE               4

#define POSE0_IDX             3
#define POSE1_IDX            12
#define POSE2_IDX            21
#define POSE3_IDX            30

#define POSE_SIZE             9


#endif // PID_REGULATOR_CONFIG_H
