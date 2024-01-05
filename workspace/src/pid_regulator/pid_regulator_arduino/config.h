#ifndef PID_REGULATOR_CONFIG_H
#define PID_REGULATOR_CONFIG_H


#define MOTOR_0_PWM_PIN       5
#define MOTOR_1_PWM_PIN       6
#define MOTOR_2_PWM_PIN       9
#define MOTOR_3_PWM_PIN      10

#define MOTOR_0_ENCA          0 // YELLOW
#define MOTOR_1_ENCA          1
#define MOTOR_2_ENCA          2
#define MOTOR_3_ENCA          3

#define MOTOR_0_ENCB          4 // GREEN
#define MOTOR_1_ENCB          7
#define MOTOR_2_ENCB          8
#define MOTOR_3_ENCB         11

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
#define START_BYTE0_CELL      0
#define START_BYTE1_CELL      1
#define TASK_CELL             2
#define ANSWER_CELL   TASK_CELL

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

#define PING_TASK             0
#define DISCONNECT_TASK       1
#define PID_TASK              2
#define RESET_PID_TASK        3
#define OK_TASK               4
#define POSE_TASK             5

#define TASKS_CNT             6


#endif // PID_REGULATOR_CONFIG_H
