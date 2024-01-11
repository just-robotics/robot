#include <Wire.h>

#include <VL53L1X.h>

VL53L1X sensor0;
VL53L1X sensor1;
VL53L1X sensor2;

int data[3];


//USE_I2C_2V8K;
void setup() {

  pinMode(2, OUTPUT); // XSHUT
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);

 // Initalisiert I2C
  delay(500);
  Wire.begin();
  Wire.beginTransmission(0x29);
  Serial.begin (9600);

  digitalWrite(4, HIGH);
  delay(150);
  sensor0.init();
  sensor0.setAddress(0x31);

  digitalWrite(5,HIGH);
  delay(150);
  sensor1.init();
  sensor1.setAddress(0x33);

  digitalWrite(6,HIGH);
  delay(150);
  sensor2.init();
  sensor2.setAddress(0x39);

  sensor0.setDistanceMode(VL53L1X::Long);
  sensor0.setMeasurementTimingBudget(50000);
  sensor0.startContinuous(50);
  sensor0.setTimeout(100);

  sensor1.setDistanceMode(VL53L1X::Long);
  sensor1.setMeasurementTimingBudget(50000);
  sensor1.startContinuous(50);
  sensor1.setTimeout(100);
  
  sensor2.setDistanceMode(VL53L1X::Long);
  sensor2.setMeasurementTimingBudget(50000);
  sensor2.startContinuous(50);
  sensor2.setTimeout(100);
  
  delay(150);
  Serial.println("addresses set");

  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;


  for (byte i = 1; i < 120; i++)
  {

    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
    {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
    } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}

void loop() {

  data[0] = sensor0.read();
  data[1] = sensor1.read();
  data[2] = sensor2.read();
  Serial.print(data[0]);
  Serial.print(" ");
  Serial.print(data[1]);
  Serial.print(" ");
  Serial.println(data[2]);
  
  delay(50);
}
