#include "config.h"
#include "ultrasound.h"


void setup() {
    Serial.begin(2000000);
}


float dist = 0.0;

void loop() {

    dist = sensor0.get_distance();
  
    Serial.println(dist);
    delay(100);
}
