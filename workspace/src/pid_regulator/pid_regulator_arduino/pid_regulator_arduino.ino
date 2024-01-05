#include "connection.h"
#include "config.h"
#include "motor.h"


uint64_t start_time;


void Connection::init() {
    Connection::callbacks = new callback[TASKS_CNT];

    Connection::callbacks[PING_TASK] = [] () {Connection::sendMessage(Connection::ping_msg, PING_SIZE, PING_TASK);};
    Connection::callbacks[DISCONNECT_TASK] = [] () {Connection::is_connected = false;};
    //Connection::callbacks[PID_TASK] = [] (uint8_t* data) {Pid::setPidVariables(data);};
    //Connection::callbacks[RESET_PID_TASK] = [] (uint8_t* data) {Pid::reset();};

    Connection::cmd_sizes = new uint64_t[TASKS_CNT];

    Connection::cmd_sizes[PING_TASK] = PING_SIZE;
    Connection::cmd_sizes[DISCONNECT_TASK] = DISCONNECT_SIZE;
    Connection::cmd_sizes[PID_TASK] = PID_CMD_SIZE;
    Connection::cmd_sizes[RESET_PID_TASK] = RESET_PID_CMD_SIZE;
}


void setup() {    
    pinMode(MOTOR_0_ENCA, INPUT); // bug with rx/tx
    pinMode(MOTOR_1_ENCA, INPUT); // bug with rx/tx
     
    Serial.begin(SERIAL_BAUDRATE);
    Serial.setTimeout(0);

    Motor::init();

    Connection::init();

    Connection::setConnection();

    motor0.set_target(200);
    motor1.set_target(200);
    motor2.set_target(200);
    motor3.set_target(200);

    start_time = millis();
}


void loop() {
    /*if (Connection::is_connected == false) {
        Connection::setConnection();
    }*/


    Motor::spinMotors();
/*
    Serial.print((int)poses[0]);
    Serial.print(" ");
    Serial.print((int)poses[1]);
    Serial.print(" ");
    Serial.print((int)poses[2]);
    Serial.print(" ");
    Serial.println((int)poses[3]);
*/
    
    //Connection::receiveCommand(PID_CMD_SIZE, Connection::callbacks[PID_TASK]);
    
    if (millis() - start_time >= 100) {
        Connection::send_poses(poses);
        start_time = millis();
    }

}
