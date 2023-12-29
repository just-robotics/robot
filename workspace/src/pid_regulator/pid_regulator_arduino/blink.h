#ifndef PID_REGULATOR_BLINK_H
#define PID_REGULATOR_BLINK_H


#define BLINK_TIMER 200


namespace Blink {
    uint8_t blink_count;
    bool blink_flag = false;
    uint64_t blink_timer;

    void blink(int cnt);
    void checkBlink();
}


void Blink::blink(int cnt) {
    
    if (blink_count == 0) {
        if (cnt > 0) {
            blink_count = cnt * 2;
            blink_flag = !blink_flag;
            digitalWrite(LED_BUILTIN, blink_flag);
            return;
        }
        digitalWrite(LED_BUILTIN, LOW);
        return;
    }
    blink_flag = !blink_flag;
    digitalWrite(LED_BUILTIN, blink_flag);
    blink_count--;
}


void Blink::checkBlink() {
    if (millis() - blink_timer > BLINK_TIMER) {
        blink(0);
        blink_timer = millis();
    }
}


#endif // PID_REGULATOR_BLINK_H
