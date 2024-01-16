#include "serial.h"
#include "config.h"
#include "motor.h"


uint64_t start_time;


void serial::set_callbacks() {
    callbacks = new callback[TASK_CNT];

    callbacks[PING_TASK] = [] (uint8_t* msg) {serial::send(msg, PING_SIZE, PING_TASK);};
    callbacks[DISCONNECT_TASK] = [] () {is_connected = false;};
    //callbacks[PID_TASK] = [] (uint8_t* data) {Pid::setPidVariables(data);};
    //callbacks[RESET_PID_TASK] = [] (uint8_t* data) {Pid::reset();};
}


void serial::set_msg_sizes() {
    msg_sizes = new uint64_t[TASK_CNT];

    msg_sizes[PING_TASK] = PING_SIZE;
    msg_sizes[DISCONNECT_TASK] = DISCONNECT_SIZE;
    msg_sizes[PID_TASK] = PID_CMD_SIZE;
    msg_sizes[RESET_PID_TASK] = RESET_PID_CMD_SIZE;
}


void setup() {    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
     
    //Serial.begin(SERIAL_BAUDRATE);
    //Serial.setTimeout(0);

    Motor::init();

    serial::init();

    serial::connect();

    motor0.set_target(200);
    motor1.set_target(200);
    motor2.set_target(200);
    motor3.set_target(200);

    start_time = millis();
}


void loop() {
    if (serial::is_connected == false) {
        serial::connect();
    }


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
        serial::send_poses(poses);
        start_time = millis();
    }

}
