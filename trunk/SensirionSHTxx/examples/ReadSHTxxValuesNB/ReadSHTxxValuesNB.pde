/**
* ReadSHT1xValuesNB
*
* Read temperature and humidity values from an SHT1x-series (SHT10,
* SHT11, SHT15) sensor in non-blocking mode.
*
* Copyright 2010 Eric C. Gionet <lavco_eg@hotmail.com>
*/

#include <SensirionSHTxx.h>

// Specify data and clock connections and instantiate SHT1x object
#define dataPin 4
#define clockPin 5
//
// instantiate SensirionSHTxx instance
SensirionSHTxx shtxx(dataPin, clockPin);
//
#define SMP_ERR 999.99
//
static float taSmp = SMP_ERR;
static float tdSmp = SMP_ERR;
static float rhSmp = SMP_ERR;

void processSHTxx()
{
  SHTxxSTATE_SENSOR result;
  
  /*
    STATE CODES (checkSensor):
      Sleeping,     // 0
      Measuring,    // 1
      Completed,    // 2
      WriteError,   // 3
      Timeout       // 4
  */
  
  // poll sensor
  result = shtxx.check(&taSmp, &rhSmp, &tdSmp);
  //
  // validate sensor result code
  if ( result == SHTxxSTATE_COMPLETED ) {
 // Print the values to the serial port
    Serial.print("Air-Temperature: ");
    Serial.print(taSmp);
    Serial.print("C. Dewpoint-Temperature: ");
    Serial.print(tdSmp);
    Serial.print("C. Humidity: ");
    Serial.print(rhSmp);
    Serial.println("%");
  } else if ( result == SHTxxSTATE_WRITE_ERROR || result == SHTxxSTATE_TIMEOUT ) {
    taSmp = SMP_ERR;
    tdSmp = SMP_ERR;
    rhSmp = SMP_ERR;
    Serial.println("Sensor Read Error/Timeout");
  }
}

void setup()
{
   Serial.begin(9600); // Open serial connection to report values to host
   Serial.println("Starting up");
   //
   // set shtxx measurement interval (3-sec)
   // and enable non-blocking
   shtxx.begin();
   shtxx.setTimerInterval( 3000 );
   shtxx.enableTimer();
}

void loop()
{
   // process sht measurement
   processSHTxx();
}
