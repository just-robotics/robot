#define PIN_TRIG 9
#define PIN_ECHO 8

#include <Wire.h>

long duration, distance;
uint8_t* data;

//Функция для получения расстояния
void SonarSensor(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(10);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  
}


void int64_to_uint8arr(int64_t number, uint8_t* output) {
    uint8_t byte = 0x000000FF;

    output[8] = number < 0 ? 0 : 1;
    uint64_t u_number = abs(number);

    output[0] = u_number & byte;
    
    for (int i = 1; i < 8; i++) {
        u_number >>= 8;
        output[i] = u_number & byte;
    }
}


void setup() {
  Wire.begin(0);                // join I2C bus with address #0
  Wire.onRequest(requestEvent); // register event
  
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

void loop() {
  SonarSensor(PIN_TRIG, PIN_ECHO);
  int64_to_uint8arr((int64_t)distance, data);
  delay(100);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
void requestEvent() {
  Wire.write(data, 9); // Отправляем массив из байтов, указываем сколько байт
}
