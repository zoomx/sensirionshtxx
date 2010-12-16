/**
* SensirionSHTxx Library
*
* Copyright 2010 Eric C. Gionet <lavco_eg@hotmail.com>
*
* Manages communication with SHTxx series (SHT10, SHT11, SHT15, SHT71, SHT75)
* temperature / humidity sensors from Sensirion (www.sensirion.com).  This
* library supports two modes of operation consisting of blocking and non-blocking
* measurement operations.  Non-blocking mode relies on a user supplied measurement
* interval which is then polled within the implementation code.
*
* Please note; when implementing hardware and using the code in a non-blocking mode
* data and clock pins must be segregated for each sensor in use.  The code can be
* easily modified to support multiple sensors having 1 clock line and multiple data lines.
*/

#ifndef SensirionSHTxx_h
#define SensirionSHTxx_h

#include "WProgram.h"

typedef enum SHTxxSTATE_SENSOR {
	SHTxxSTATE_SLEEPING,
	SHTxxSTATE_MEAS_TEMP,
	SHTxxSTATE_TEMP_CHECK,
	SHTxxSTATE_MEAS_HUMI,
	SHTxxSTATE_HUMI_CHECK,
	SHTxxSTATE_COMPLETED, 
	SHTxxSTATE_WRITE_ERROR,
	SHTxxSTATE_TIMEOUT,
	SHTxxSTATE_DISABLED
};

typedef enum SHTxxMODE_MEASURE {
	SHTxxMODE_TEMP,
	SHTxxMODE_HUMI
};

class SensirionSHTxx
{
  	public:
		// constructors
		//SensirionSHTxx(unsigned char clockPin);
		SensirionSHTxx(uint8_t dataPin, uint8_t clockPin);

		// initialization functions
		void begin(void);
		void begin(uint32_t intervalMillis);

		// non-blocking related functions
		void setTimerInterval(uint32_t intervalMillis);
		void enableTimer(void);
		void disableTimer(void);
		void resetTimer(void);
		SHTxxSTATE_SENSOR check(float *ta, float *rh, float *td);
		SHTxxSTATE_SENSOR check(float *ta, float *rh);
		SHTxxSTATE_SENSOR check(float *ta);

		// blocking related functions
	    	uint8_t read(float *ta, float *rh, float *td);
		uint8_t read(float *ta, float *rh);
		uint8_t read(float *ta);
	    	bool reset(void);
		bool heaterOn(void);
		bool heaterOff(void);
		bool reloadEnable(void);
		bool reloadDisable(void);
		uint8_t getStatusRegister(void);  // remove ??

		// general functions
	    	float convertCtoF(float ta);
		float calcAbsHumidity(float rh, float ta);
		float calcHeatIndex(float rh, float ta);

	private:
		uint8_t _dataPin;
	    	uint8_t _clockPin;
		uint32_t _previousMillis;
		uint32_t _intervalMillis;
		uint8_t _status;
		SHTxxSTATE_SENSOR _state;
		bool _active;
		uint16_t _rhRaw;
		uint16_t _taRaw;

		// helper functions
		uint8_t intervalCheck(void);
		float convertTa(uint16_t taRaw);
		float convertRh(uint16_t rhRaw, float ta);
		float calcTd(float rh, float ta);
	    	uint8_t writeByte(uint8_t value);
	    	uint8_t readByte(uint8_t ack);
	    	void transmissionStart(void);
	    	void connectionReset(void);
    		uint8_t softReset(void);
    		uint8_t measure(uint16_t *value, SHTxxMODE_MEASURE mode);
		uint8_t readStatus(uint8_t *status);
		uint8_t writeStatus(uint8_t status);
};

#endif
