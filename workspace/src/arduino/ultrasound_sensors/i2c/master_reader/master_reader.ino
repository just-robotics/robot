#include <Wire.h>

uint8_t* my_data;
int64_t distance;

int64_t uint8arr_to_int64(uint8_t* data) {
    int64_t number = data[7];

    for (int i = 6; i >= 0; i--) {
        number <<= 8;
        number = number | data[i];
    }

    number = data[8] == 1 ? number : -number;
    
    return number;
}
 

void setup() {
  Wire.begin();        // join I2C bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop() {
  Wire.requestFrom(0, 9);    // request 9 bytes from slave device #0

  while (Wire.available()) { // slave may send less than requested
    my_data = Wire.read(); // receive a byte as character
    distance = uint8arr_to_int64(my_data);
    Serial.println((long)distance);         // print the data
  }

  delay(100);
}
