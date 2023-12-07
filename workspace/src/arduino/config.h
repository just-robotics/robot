
#ifndef Config_h
#define Config_h


#include <stdint.h>


#define SERIAL_BAUDRATE       9600

#define M1_DIR                   4
#define M1_PWM                   5
#define M2_DIR                   7
#define M2_PWM                   6

#define LEFT                     0
#define RIGHT                    1
#define FORWARD                  1
#define BACKWARD                 0

#define DEFAULT_SPEED          255
#define DEFAULT_ROTATE_SPEED   255

#define TIMER                  100
#define START_BYTE              64

#define COMMAND_START_BYTE1_CELL 0
#define COMMAND_START_BYTE2_CELL 1
#define COMMAND_TASK_CELL        2
#define COMMAND_VALUE1_CELL      3
#define COMMAND_VALUE2_CELL      4
#define COMMAND_CHECKSUM_CELL    5
#define COMMAND_SIZE             6

#define MESSAGE_START_BYTE1_CELL 0
#define MESSAGE_START_BYTE2_CELL 1
#define MESSAGE_ANSWER_CELL      2
#define MESSAGE_CHECKSUM_CELL    3
#define MESSAGE_SIZE             4

#define PING_TASK                0
#define PING_VALUE1              0
#define PING_VALUE2              0

#define MOVE_BACKWARD_TASK       1
#define MOVE_FORWARD_TASK        2
#define STOP_TASK                3
#define TURN_RIGHT_TASK          4
#define TURN_LEFT_TASK           5
#define LED_ON_TASK              6
#define LED_OFF_TASK             7


#endif
