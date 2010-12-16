/**
* ReadSHT1xValues
*
* Read temperature and humidity values from an SHT1x-series (SHT10,
* SHT11, SHT15) sensor.
*
* Copyright 2010 Eric C. Gionet <lavco_eg@hotmail.com>
*/

#include <SensirionSHTxx.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin 4
#define clockPin 5
SensirionSHTxx shtxx(dataPin, clockPin);

void setup()
{
   Serial.begin(9600); // Open serial connection to report values to host
   Serial.println("Starting up");
}

void loop()
{
  float ta;
  float rh;

  // Read values from the sensor
  shtxx.readSensor(&ta, &rh);

  // Print the values to the serial port
  Serial.print("Temperature: ");
  Serial.print(ta);
  Serial.print("C. Humidity: ");
  Serial.print(rh);
  Serial.println("%");
  
  // Delay 5-seconds before next measurement
  delay(5000);
}
