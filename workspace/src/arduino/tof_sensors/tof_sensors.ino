#include <Wire.h>

#include <VL53L1X.h>


#define size 2

VL53L1X sensors[size];

int xshuts[] = {4, 5};
int addr[] = {0x31, 0x33};

int data[size];

bool nop_flag;


void nop() {
    return;
}


void setup() {
  
    for (uint8_t i = 0; i < size; i++) {
        pinMode(xshuts[i], OUTPUT);
        digitalWrite(xshuts[i], LOW);
    }

    Wire.begin();
    Wire.beginTransmission(0x29);
    Serial.begin (9600);

    for (uint8_t i = 0; i < size; i++) {
        digitalWrite(xshuts[i], HIGH);
        delay(150);
        sensors[i].init();
        sensors[i].setAddress(addr[i]);
    }
    Serial.println("addresses set");

    for (uint8_t i = 0; i < size; i++) {
        sensors[i].setDistanceMode(VL53L1X::Long);
        sensors[i].setMeasurementTimingBudget(50000);
        sensors[i].startContinuous(50);
        sensors[i].setTimeout(100);
    }
  
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 120; i++) {
      Wire.beginTransmission(i);
      if (Wire.endTransmission() == 0) {
          Serial.print ("Found address: ");
          Serial.print (i, DEC);
          Serial.print (" (0x");
          Serial.print (i, HEX);
          Serial.println (")");
          count++;
          delay (1);
      }
  }
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
  nop_flag = !count;
}

void loop() {
    if (nop_flag) {
        nop();
    }
    else {
        for (int i = 0; i < size; i++) {
            data[i] = sensors[i].read();
            Serial.print(data[i]);
            Serial.print(" ");
        }
        Serial.println();
        delay(50);
    }
}
