/**
* SensirionSHTxx Library
*
* Copyright 2010 Eric Gionet <lavco_eg@hotmail.com>
*
* Manages communication with SHTxx series (SHT10, SHT11, SHT15, SHT71, SHT75)
* temperature / humidity sensors from Sensirion (www.sensirion.com).
*/
#include "WProgram.h"
#include "SensirionSHTxx.h"

/********************************************************************
 * CONFIGURATION
 ********************************************************************/

#define SENSIRIONSHTxx_VERSION 1001
#define SENSIRIONSHTxx_VERSION_STRING "1.1"

// sensor setup - set as needed
#define SHT_RES		 1		// 1 (High) or 0 (Low) - Resolution
#define SHT_VOLT	 5		// 5 or 3.5 (Volt) - Operating Voltage

/******************************************************************************
 * Definitions
 ******************************************************************************/

// register dr command r/w
#define SHT_REG_STATUS_W 0x06 		//000 0011 0
#define SHT_REG_STATUS_R 0x07 		//000 0011 1
#define SHT_REG_RESET    0x1e 		//000 1111 0
#define SHT_REG_TEMP	 0x03 		//000 0001 1
#define SHT_REG_HUMI     0x05 		//000 0010 1
#define SHT_ACK		 1
#define SHT_noACK	 0
// humidity conversion coefficients
#if SHT_RES == 1
#define SHT_C1		-2.0468		// for 12 Bit
#define SHT_C2		+0.0367		// for 12 Bit
#define SHT_C3		-0.0000015955 	// for 12 Bit
#else
#define SHT_C1		-2.0468		// for 8 Bit
#define SHT_C2		+0.5872		// for 8 Bit
#define SHT_C3		-0.00040845 	// for 8 Bit
#endif
// temperature compensation coefficients
#if SHT_RES == 1
#define SHT_T1		+0.01  		// for 12 Bit
#define SHT_T2		+0.00008	// for 12 Bit
#else
#define SHT_T1		+0.01    	// for 8 Bit
#define SHT_T2		+0.00128	// for 8 Bit
#endif
// temperature conversion coefficients
#if SHT_VOLT == 5
#define SHT_D1		-40.1		// for degC @ 5V
#else
#define SHT_D1		-39.7		// for degC @ 3.5V
#endif
#if SHT_RES == 1
#define SHT_D2		0.01		// for 14 bit degC
#else
#define SHT_D2		0.04		// for 12 bit degC
#endif
// pulse lengths
#define SHT_PULSE_LONG  delayMicroseconds(3) 
#define SHT_PULSE_SHORT delayMicroseconds(1)

/*
// multi-sensor measurements
#define SHT_MAX_SENSORS 2

typedef struct SHTxx_SENSOR {
	SHTxxSTATE_SENSOR state;
	unsigned char dataPin; 
	unsigned int taRaw;
	unsigned int rhRaw;
	float ta;
	float td;
	float rh;
};

typedef struct SHTxx_SENSORS {
	unsigned char index;
	unsigned char count;
	SHTxx_SENSOR entries[SHT_MAX_SENSORS];
};
*/

/* ================ Class Constructors ================ */

//SensirionSHTxx::SensirionSHTxx(unsigned char clockPin)
//{
  	// set class properties
//  	_clockPin = clockPin;
//}

SensirionSHTxx::SensirionSHTxx(uint8_t dataPin, uint8_t clockPin)
{
  	// set class properties
  	_dataPin = dataPin;
  	_clockPin = clockPin;
}

/* ================ Public methods ================ */

void SensirionSHTxx::begin(void)
{
	// set DDR for software SPI pins
	pinMode(_clockPin, OUTPUT);
	pinMode(_dataPin, OUTPUT);
	//
	// issue sensor soft reset
	softReset();
	//
	// stats register is reset, if
	// low resolution is set, write
	// resgister status to sensor
	if ( SHT_RES == 0 ) {
		// mask read status
		_status = _status | 00000001;
		//
		// write masked status to register
		writeStatus(_status);
	}
}

void SensirionSHTxx::begin(uint32_t intervalMillis)
{
	// call default begin
	// to init class
	begin();
	//
	// set measurement interval
	// and enable
	setTimerInterval(intervalMillis);
	enableTimer();
}

// Purpose:
// Sets interval when measurements take place
// for non-blocking calls.  Minimum interval
// is 1/2-sec however when measuring two parameters
// in high resolution mode the interval should not
// be less than 1-sec to prevent internal heating of
// the sensor.
//
// Params:
// intervalMillis - Interval in milliseconds
//
// Return:
// Nothing.
void SensirionSHTxx::setTimerInterval(uint32_t intervalMillis)
{
	// minimum polling is 1/2-second
	if ( intervalMillis < 500 ) {
		_intervalMillis = 500;
	} else {
		_intervalMillis = intervalMillis;
	}
}

void SensirionSHTxx::resetTimer(void) 
{
	// reset state
	_state = SHTxxSTATE_SLEEPING;
	//
	_previousMillis = millis();
}

void SensirionSHTxx::disableTimer(void)
{
	_state = SHTxxSTATE_DISABLED;
	_active = false;
}

void SensirionSHTxx::enableTimer(void)
{
	_state = SHTxxSTATE_SLEEPING;
	_active = true;
	//
	resetTimer();
}

SHTxxSTATE_SENSOR SensirionSHTxx::check(float *ta, float *rh, float *td)
{
	/*
	enum SHTxxSTATE_SENSOR {
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
	*/

	// check sensor and calculate dew-point if completed
	if ( check(ta, rh) == SHTxxSTATE_COMPLETED ) {
		*td = calcTd(*rh, *ta);
		return SHTxxSTATE_COMPLETED;
	}

	return _state;
}

SHTxxSTATE_SENSOR SensirionSHTxx::check(float *ta, float *rh)
{
	uint8_t i = 0;

	/*
	enum SHTxxSTATE_SENSOR {
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
	*/

	// non-blocking is disabled
	if ( _active == false ) return _state;
	//
	// should we poll the sensor
	if ( intervalCheck() ) {
		// temperature and humidity delay - 400ms
		// 20/80/320 ms for a 8/12/14bit
		// 14bit(temp) + 12bit(rh) = 320 + 80 = 400ms
		if ( millis() - _previousMillis > 400 + _intervalMillis ) {
			// reset timer interval
			_previousMillis += _intervalMillis;
			//
			// reset state
			_state = SHTxxSTATE_SLEEPING;
			//
			// timeout
			return SHTxxSTATE_TIMEOUT;
		}
		//
		// validate status mode
		if ( _state == SHTxxSTATE_MEAS_TEMP ) {
			// start temperature measurement
			//
			transmissionStart(); //transmission start
			//
			if ( writeByte(SHT_REG_TEMP) ) {
				// reset timer interval
				_previousMillis += _intervalMillis;
				//
				// reset state
				_state = SHTxxSTATE_SLEEPING;
				//
				// write error
				return SHTxxSTATE_WRITE_ERROR;
			}
			//
			// set state to check
			// temperature measurement status
			_state = SHTxxSTATE_TEMP_CHECK;
		} else if ( _state == SHTxxSTATE_TEMP_CHECK ) {
			// check temperature measurement status
			//
		        if ( digitalRead(_dataPin) == 0 ){
				// reset raw temperature
				_taRaw = 0;
				//
				i = readByte(1); //read the first byte (MSB)
				_taRaw = (i << 8) | readByte(0); //read the second byte (LSB) and end with no-ack
				//
				// convert temperature
				*ta = convertTa(_taRaw);
				//
				// set state mode to start
				// humidity measurement
				_state = SHTxxSTATE_MEAS_HUMI;
		        }
		} else if ( _state == SHTxxSTATE_MEAS_HUMI ) {
			// start humidity measurement
			//
			transmissionStart(); //transmission start
			//
			if ( writeByte(SHT_REG_HUMI) ) {
				// reset timer interval
				_previousMillis += _intervalMillis;
				//
				// reset state
				_state = SHTxxSTATE_SLEEPING;
				//
				// write error
				return SHTxxSTATE_WRITE_ERROR;
			}
			//
			// set state mode to check
			// humidity measurement status
			_state = SHTxxSTATE_HUMI_CHECK;
		} else if ( _state == SHTxxSTATE_HUMI_CHECK ) {
			// check humidity measurement status
			//
		        if ( digitalRead(_dataPin) == 0 ){
				// reset raw humidity
				_rhRaw = 0;
				//
				i = readByte(1); //read the first byte (MSB)
				_rhRaw = (i << 8) | readByte(0); //read the second byte (LSB) and end with no-ack
				//
				// convert humdity
				*rh = convertRh(_rhRaw, *ta);
				//
				// reset timer interval
				_previousMillis += _intervalMillis;
				//
				// reset state
				_state = SHTxxSTATE_SLEEPING;
				//
				// set return state to completed
				return SHTxxSTATE_COMPLETED;
		        }
		}
	}

	return _state;
}

SHTxxSTATE_SENSOR SensirionSHTxx::check(float *ta)
{
	uint8_t i = 0;

	/*
	enum SHTxxSTATE_SENSOR {
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
	*/

	// non-blocking is disabled
	if ( _active == false ) return SHTxxSTATE_DISABLED;
	//
	// should we poll the sensor
	if ( intervalCheck() ) {
		// temperature and humidity delay 
		// 20/80/320 ms for a 8/12/14bit
		// 14bit(temp) = 320ms
		if ( millis() - _previousMillis > 320 + _intervalMillis ) {
			// reset timer interval
			_previousMillis += _intervalMillis;
			//
			// reset state
			_state = SHTxxSTATE_SLEEPING;
			//
			// timeout
			return SHTxxSTATE_TIMEOUT;
		}
		//
		// validate status mode
		if ( _state == SHTxxSTATE_MEAS_TEMP ) {
			// start temperature measurement
			//
			transmissionStart(); //transmission start
			//
			if ( writeByte(SHT_REG_TEMP) ) {
				// reset timer interval
				_previousMillis += _intervalMillis;
				//
				// reset state
				_state = SHTxxSTATE_SLEEPING;
				//
				// write error
				return SHTxxSTATE_WRITE_ERROR;
			}
			//
			// set state to check
			// temperature measurement
			_state = SHTxxSTATE_TEMP_CHECK;
		} else if ( _state == SHTxxSTATE_TEMP_CHECK ) {
			// check temperature measurement status
			//
		        if ( digitalRead(_dataPin) == 0 ){
				// reset raw temperature
				_taRaw = 0;
				//
				i = readByte(1); //read the first byte (MSB)
				_taRaw = (i << 8) | readByte(0); //read the second byte (LSB) and end with no-ack
				//
				// convert temperature
				*ta = convertTa(_taRaw);
				//
				// reset timer interval
				_previousMillis += _intervalMillis;
				//
				// reset state
				_state = SHTxxSTATE_SLEEPING;
				//
				// set result to completed
				return SHTxxSTATE_COMPLETED;
		        }
		} 
	}

	return _state;
}


uint8_t SensirionSHTxx::read(float *ta, float *rh, float *td)
{
	uint8_t error = 0;

	// check if non-blocking is active
	if ( _active ) return 3;
	//
	// measure temperature and humidity
	if ((error = read(ta, rh)) != 0)
		return(error);
	//
	// calculate dew-point
	*td = calcTd(*rh, *ta);
	
	return(error);
}

uint8_t SensirionSHTxx::read(float *ta, float *rh)
{
	uint8_t error = 0;

	// reset raw humidity
	_rhRaw = 0;
	//
	// measure temperature
	if ((error = read(ta)) != 0)
		return(error);
	//
	// measure raw humidity [Ticks] 
	// 12 Bit (Hi) | 8 Bit (Lo)
	if ((error = measure(&_rhRaw, SHTxxMODE_HUMI)) != 0)
		return(error);
	//
	// convert humidity
	*rh = convertRh(_rhRaw, *ta);
	
	return(error);
}

uint8_t SensirionSHTxx::read(float *ta)
{
	uint8_t error = 0;

	// reset raw temperature
	_taRaw = 0;
	//
	// check if non-blocking is active
	if ( _active ) return 3;
	//
	// measure raw temperature [Ticks] 
	// 14 Bit (Hi) | 12 Bit (Lo)
	if ((error = measure(&_taRaw, SHTxxMODE_TEMP)) != 0)
		return(error);
	//
	// convert temperature
	*ta = convertTa(_taRaw);
	
	return(error);
}

bool SensirionSHTxx::reset(void)
{
	// check if non-blocking is active
	if ( _active ) return false;  // ?? disable, reset, then enable
	//
	// reset the sensor
	if ( softReset() == 0 ) return true;
	//
	return false;
}

bool SensirionSHTxx::heaterOn(void)
{
	// check if non-blocking is active
	if ( _active ) return false;
	//
	// read status register
	if ( readStatus(&_status) != 0 ) return false;
	//
	// mask read status
	_status = _status | 00000100;
	//
	// write masked status to register
	if ( writeStatus(_status) != 0 ) return false;
	//
	return true;
}

bool SensirionSHTxx::heaterOff(void)
{
	// check if non-blocking is active
	if ( _active ) return false;
	//
	// read status register
	if ( readStatus(&_status) != 0 ) return false;
	//
	// mask read status
	_status = _status & 11110111;
	//
	// write masked status to register
	if ( writeStatus(_status) != 0 ) return false;
	//
	return true;
}

bool SensirionSHTxx::reloadEnable(void)
{
	// check if non-blocking is active
	if ( _active ) return false;
	//
	// read status register
	if ( readStatus(&_status) != 0 ) return false;
	//
	// mask read status
	_status = _status | 00000010;
	//
	// write masked status to register
	if ( writeStatus(_status) != 0 ) return false;
	//
	return true;
}

bool SensirionSHTxx::reloadDisable(void)
{
	// check if non-blocking is active
	if ( _active ) return false;
	//
	// read status register
	if ( readStatus(&_status) != 0 ) return false;
	//
	// mask read status
	_status = _status & 11111101;
	//
	// write masked status to register
	if ( writeStatus(_status) != 0 ) return false;
	//
	return true;
}

uint8_t SensirionSHTxx::getStatusRegister(void)
{
	return _status;
}

float SensirionSHTxx::convertCtoF(float ta)
{
	// calculate and return value
	return (ta*1.8+32);
}

float SensirionSHTxx::calcAbsHumidity(float rh, float ta)
{
	return 216.7*(rh/100.0*6.112*exp(17.62*ta/(243.12+ta))/(273.15+ta));
}

float SensirionSHTxx::calcHeatIndex(float rh, float ta)
{
	float p = rh/100.0*exp(17.62*ta/(243.12+ta));
	//
	return ta+5.0/9.0*(p-10.0);
}

/* ================ Private methods ================ */

uint8_t SensirionSHTxx::intervalCheck(void)
{
	if (millis() - _previousMillis >= _intervalMillis) {
		if ( _state == SHTxxSTATE_SLEEPING ) _state = SHTxxSTATE_MEAS_TEMP; // start temperature measurement

		return 1;
	}

  	return 0;
}

float SensirionSHTxx::convertTa(uint16_t taRaw)
{
	// calc. temperature from ticks to [°C]
	// and return converted value
	return SHT_D1 + SHT_D2 * taRaw;
}

float SensirionSHTxx::convertRh(uint16_t rhRaw, float ta)
{
	// calc. humidity from ticks to [%RH]
	float rhLin = SHT_C3 * rhRaw * rhRaw + SHT_C2 * rhRaw + SHT_C1;
	//
	// calc. temperature compensated humidity [%RH]
	// and return calculated value
	return (ta - 25 ) * (SHT_T1 + SHT_T2 * rhRaw) + rhLin;
}

float SensirionSHTxx::calcTd(float rh, float ta)
{
	//float logEx = 0.66077 + 7.5 * ta / (237.3 + ta) + (log10(rh) - 2);
	//
	//return (logEx - 0.66077) * 237.3 / (0.66077 + 7.5 - logEx);

	float logEx = (log10(rh)-2.0)/0.4343+(17.62*ta)/(243.12+ta);
	//
	return 243.12*logEx/(17.62-logEx);
}

// writes a byte on the Sensibus and checks the acknowledge
uint8_t SensirionSHTxx::writeByte(uint8_t value)
{
        uint8_t i = 0x80;
        uint8_t error = 0;

        // Set data line for output
        pinMode(_dataPin, OUTPUT);
        while(i){ //shift bit for masking
                if (i & value) {
                        digitalWrite(_dataPin, HIGH); //masking value with i , write to SENSI-BUS
                }else{ 
                        digitalWrite(_dataPin, LOW);
                }
                digitalWrite(_clockPin, HIGH); //clk for SENSI-BUS
                SHT_PULSE_LONG;
                digitalWrite(_clockPin, LOW);
                SHT_PULSE_SHORT;
                i=(i>>1);
        }
        pinMode(_dataPin, INPUT); //release DATA-line
        digitalWrite(_clockPin, HIGH); //clk #9 for ack
        //SHT_PULSE_LONG; // ?? PULSE SHORT as per sensirion sample code
	SHT_PULSE_SHORT;
        if (digitalRead(_dataPin)){ //check ack (DATA will be pulled down by SHT11)
                error = 1;
        }
        SHT_PULSE_SHORT;
        digitalWrite(_clockPin, LOW);
        return(error); //error=1 in case of no acknowledge
}

// Reads a byte form the Sensibus and returns an acknowledge in case of "ack=1"
uint8_t SensirionSHTxx::readByte(uint8_t ack)
{
        uint8_t i = 0x80;
        uint8_t val = 0;

        pinMode(_dataPin, INPUT); //release DATA-line
        //
        while(i){ //shift bit for masking
                digitalWrite(_clockPin, HIGH); //clk for SENSI-BUS
                SHT_PULSE_SHORT;
                if (digitalRead(_dataPin)){
                        val=(val | i); //read bit
                }
                digitalWrite(_clockPin, LOW);
                SHT_PULSE_SHORT;
                i=(i>>1);
        }
        pinMode(_dataPin, OUTPUT);
        if (ack){
                //in case of "ack==1" pull down DATA-Line
                digitalWrite(_dataPin, LOW);
        }else{
                digitalWrite(_dataPin, HIGH);
        }
        digitalWrite(_clockPin, HIGH); //clk #9 for ack
        SHT_PULSE_LONG;
        digitalWrite(_clockPin, LOW);
        SHT_PULSE_SHORT;
        pinMode(_dataPin, INPUT);
        //
        return (val);
}

// generates a sensirion specific transmission start
// This is the point where sensirion is not I2C standard conform and the
// main reason why the AVR TWI hardware support can not be used.
//       _____         ________
// DATA:      |_______|
//           ___     ___
// SCK : ___|   |___|   |______
void SensirionSHTxx::transmissionStart(void)
{      
        digitalWrite(_clockPin, LOW);
        pinMode(_dataPin, OUTPUT); 
        digitalWrite(_dataPin, HIGH);
        SHT_PULSE_SHORT;
        digitalWrite(_clockPin, HIGH);
        SHT_PULSE_SHORT;
        digitalWrite(_dataPin, LOW);
        SHT_PULSE_SHORT;
        digitalWrite(_clockPin, LOW);
        SHT_PULSE_LONG;
        digitalWrite(_clockPin, HIGH);
        SHT_PULSE_SHORT;
        digitalWrite(_dataPin, HIGH);
        SHT_PULSE_SHORT;
        digitalWrite(_clockPin, LOW);
        SHT_PULSE_SHORT;
        //
        pinMode(_dataPin, INPUT);
}

// Communication reset 
// DATA-line=1 and at least 9 SCK cycles followed by transstart
//      _____________________________________________________         ________
// DATA:                                                     |_______|
//          _    _    _    _    _    _    _    _    _        ___    ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|  |___|   |______
void SensirionSHTxx::connectionReset(void)
{
        uint8_t i;
        
        digitalWrite(_clockPin, LOW);
        pinMode(_dataPin, OUTPUT);
        digitalWrite(_dataPin, HIGH);
        
        for (i = 0; i < 9; i++) {
                digitalWrite(_clockPin, HIGH);
		SHT_PULSE_SHORT;
                digitalWrite(_clockPin, LOW);
		SHT_PULSE_SHORT;
        }

        transmissionStart();
}

// Resets the sensor by a soft reset
uint8_t SensirionSHTxx::softReset(void)
{
	// reset communication
	connectionReset();
	//
	//send RESET-command to sensor, return = 1 in case of no response
	if ( writeByte(SHT_REG_RESET) ) return(1);
	//
	// delay 10ms after a reset
	// before sending another cmd
	// as per sensirion datasheet
	delay(10);

	return(0);
}

// makes a measurement (humidity/temperature)
// value returns 2 bytes
// Modes: 1 = humidity  0 = temperature
// Return value: 1 = write error, 2 = timeout
uint8_t SensirionSHTxx::measure(uint16_t *value, SHTxxMODE_MEASURE mode)
{
        uint8_t i = 0;

	// alert sensor
        transmissionStart();
        *value = 0;
	switch( mode ) {
		case SHTxxMODE_TEMP:
			if ( writeByte(SHT_REG_TEMP) ) return(1);
			break;
		case SHTxxMODE_HUMI:
			if ( writeByte(SHT_REG_HUMI) ) return(1);
			break;
		default:
			return(1);
	}
        // normal delays: temp i=70, humi i=20
        while ( i < 240 ){
                delay(3);
                if ( digitalRead(_dataPin) == 0 ){
                        i = 0;
                        break;
                }
                i++;
        }
        if ( i ) return(2);
        i = readByte(SHT_ACK); //read the first byte (MSB)
        *value = (i << 8) | readByte(SHT_noACK); //read the second byte (LSB) and end with no-ack
        return(0);
}

// 0 = success, 1 = write error
uint8_t SensirionSHTxx::readStatus(uint8_t *status)
{
	// alert sensor
	transmissionStart();
	//
	// read status register command
	if ( writeByte(SHT_REG_STATUS_R) ) return(1);
	//
	// one byte read - no ack
	*status = readByte(SHT_noACK);
	//
	return(0);
}

// 0 = success, 1 = write error
uint8_t SensirionSHTxx::writeStatus(uint8_t status)
{
	// alert sensor
	transmissionStart();
	//
	// write status register command
	if ( writeByte(SHT_REG_STATUS_W) ) return(1);
	//
	// write status command
	if ( writeByte(status) ) return(1);
}
