 /*
  The MIT License (MIT)
  Copyright (c) 2019 Kris Kasrpzak
  Permission is hereby granted, free of charge, to any person obtaining a copy of
  this software and associated documentation files (the "Software"), to deal in
  the Software without restriction, including without limitation the rights to
  use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
  the Software, and to permit persons to whom the Software is furnished to do so,
  subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
  FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
  COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
  IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
  On a personal note, if you develop an application or product using this library 
  and make millions of dollars, I'm happy for you!
*/

/* 
  Code by Kris Kasprzak kris.kasprzak@yahoo.com
  This library is intended to be used with EBYTE transcievers, small wireless units for MCU's such as
  Teensy and Arduino. This library let's users program the operating parameters and both send and recieve data.
  This company makes several modules with different capabilities, but most #defines here should be compatible with them
  All constants were extracted from several data sheets and listed in binary as that's how the data sheet represented each setting
  Hopefully, any changes or additions to constants can be a matter of copying the data sheet constants directly into these #defines
  Usage of this library consumes around 970 bytes
  Revision		Data		Author			Description
  1.0			3/6/2019	Kasprzak		Initial creation
  2.0			3/2/2020	Kasprzak		Added all functions to build the options bit (FEC, Pullup, and TransmissionMode
  3.0			3/27/2020	Kasprzak		Added more Get functions
  4.0			6/23/2020	Kasprzak		Added private method to clear the buffer to ensure read methods would not be filled with buffered data
  5.0			12/4/2020	Kasprzak		moved Reset to public, added Clear to SetMode to avoid buffer corruption during programming
  5.5			1/26/2022	Kasprzak		added attempt parameter to limit the retries

  Module connection
  Module	MCU						Description
  MO		Any digital pin*		pin to control working/program modes
  M1		Any digital pin*		pin to control working/program modes
  Rx		Any digital pin			pin to MCU TX pin (module transmits to MCU, hence MCU must recieve data from module
  Tx		Any digital pin			pin to MCU RX pin (module transmits to MCU, hence MCU must recieve data from module
  AUX		Any digital pin			pin to indicate when an operation is complete (low is busy, high is done)
  Vcc		+3v3 or 5V0				
  Vcc		Ground					Ground must be common to module and MCU		
  notes:
  * caution in connecting to Arduino pin 0 and 1 as those pins are for USB connection to PC
  you may need a 4K7 pullup to Rx and AUX pins (possibly Tx) if using and Arduino
  Module source
  http://www.ebyte.com/en/
  example module this library is intended to be used with
  http://www.ebyte.com/en/product-view-news.aspx?id=174
  Code usage
  1. Create a serial object
  2. Create EBYTE object that uses the serail object
  3. begin the serial object
  4. init the EBYTE object
  5. set parameters (optional but required if sender and reciever are different)
  6. send or listen to sent data
  
*/

#ifndef EBYTE_H_LIB
#define EBYTE_H_LIB

#define EBYTE_H_LIB_VER 5.5

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


// if you seem to get "corrupt settings add this line to your .ino
// #include <avr/io.h>

/* 

if modules don't seem to save or read parameters, it's probably due to slow pin changing times
in the module. I see this happen rarely. You will have to adjust this value
when settin M0 an M1 there is gererally a short time for the transceiver modules
to react. The data sheet says 2 ms, but more time is generally needed. I'm using
50 ms below and maybe too long, but it seems to work in most cases. Increase this value
if your unit will not return parameter settings.
*/

#define PIN_RECOVER 50 

// modes NORMAL send and recieve for example
#define MODE_NORMAL 0			// can send and recieve
#define MODE_WAKEUP 1			// sends a preamble to waken receiver
#define MODE_POWERDOWN 2		// can't transmit but receive works only in wake up mode
#define MODE_PROGRAM 3			// for programming

// options to save change permanently or temp (power down and restart will restore settings to last saved options
#define PERMANENT 0xC0
#define TEMPORARY 0xC2

// parity bit options (must be the same for transmitter and reveiver)
#define PB_8N1 0b00			// default
#define PB_8O1 0b01
#define PB_8E1 0b11

//UART data rates
// (can be different for transmitter and reveiver)
#define UDR_1200 0b000		// 1200 baud
#define UDR_2400 0b001		// 2400 baud
#define UDR_4800 0b010		// 4800 baud
#define UDR_9600 0b011		// 9600 baud default
#define UDR_19200 0b100		// 19200 baud
#define UDR_38400 0b101		// 34800 baud
#define UDR_57600 0b110		// 57600 baud
#define UDR_115200 0b111	// 115200 baud

// air data rates (certian types of modules)
// (must be the same for transmitter and reveiver)
#define ADR_300 0b000		// 300 baud
#define ADR_1200 0b001		// 1200 baud
#define ADR_2400 0b010		// 2400 baud
#define ADR_4800 0b011		// 4800 baud
#define ADR_9600 0b100		// 9600 baud
#define ADR_19200 0b101		// 19200 baud

// air data rates (other types of modules)
#define ADR_1K 0b000		// 1k baud
#define ADR_2K 0b001		// 2K baud
#define ADR_5K 0b010		// 4K baud
#define ADR_8K 0b011		// 8K baud
#define ADR_10K 0b100		// 10K baud
#define ADR_15K 0b101		// 15K baud
#define ADR_20K 0b110		// 20K baud
#define ADR_25K 0b111		// 25K baud

// various options
// (can be different for transmitter and reveiver)
#define OPT_FMDISABLE 0b0	//default
#define OPT_FMENABLE 0b1
#define OPT_IOOPENDRAIN 0b0	 
#define OPT_IOPUSHPULL  0b1
#define OPT_WAKEUP250  0b000 
#define OPT_WAKEUP500  0b001
#define OPT_WAKEUP750  0b010
#define OPT_WAKEUP1000 0b011
#define OPT_WAKEUP1250 0b100
#define OPT_WAKEUP1500 0b101
#define OPT_WAKEUP1750 0b110
#define OPT_WAKEUP2000 0b111
#define OPT_FECDISABLE  0b0
#define OPT_FECENABLE 0b1	

// transmitter output power--check government regulations on legal transmit power
// refer to the data sheet as not all modules support these power levels
// constants for 1W units
// (can be different for transmitter and reveiver)
#define OPT_TP30 0b00		// 30 db
#define OPT_TP27 0b01		// 27 db
#define OPT_TP24 0b10		// 24 db
#define OPT_TP21 0b11		// 21 db

// constants or 500 mW units
//#define OPT_TP27 0b00		// 27 db
//#define OPT_TP24 0b01		// 24 db
//#define OPT_TP21 0b10		// 21 db
//#define OPT_TP18 0b11		// 17 db
//#define OPT_TP17 0b11		// 17 db

// constants or 100 mW units
// #define OPT_TP20 0b00		// 20 db
// #define OPT_TP17 0b01		// 17 db
// #define OPT_TP14 0b10		// 14 db
// #define OPT_TP11 0b11		// 10 db
// #define OPT_TP10 0b11		// 10 db



class Stream;

class EBYTE {

public:

	EBYTE(Stream *s, uint8_t PIN_M0 = 4, uint8_t PIN_M1 = 5, uint8_t PIN_AUX = 6);

	// code to initialize the library
	// this method reads all parameters from the module and stores them in memory
	// library modifications could be made to only read upon a change at a savings of 30 or so bytes
	// the issue with these modules are some parameters are a collection of several options AND
	// ALL parameters must be sent even if only one option is changed--hence get all parameters initially
	// so you know what the non changed parameters are know for resending back

	bool init(uint8_t _Attempts = 5);
	
	// methods to set modules working parameters NOTHING WILL BE SAVED UNLESS SaveParameters() is called
	void SetMode(uint8_t mode = MODE_NORMAL);
	void SetAddress(uint16_t val = 0);
	void SetAddressH(uint8_t val = 0);
	void SetAddressL(uint8_t val = 0);
	void SetAirDataRate(uint8_t val);
	void SetUARTBaudRate(uint8_t val);
	void SetSpeed(uint8_t val);
	void SetOptions(uint8_t val);
	void SetChannel(uint8_t val);
	void SetParityBit(uint8_t val);
	
	//functions to set the options
	void SetTransmissionMode(uint8_t val);
	void SetPullupMode(uint8_t val);
	void SetWORTIming(uint8_t val);
	void SetFECMode(uint8_t val);
	void SetTransmitPower(uint8_t val);

	bool GetAux();

	bool available();
	void flush();
	// methods to get some operating parameters
	uint16_t GetAddress();

	// methods to get module data
	uint8_t GetModel();
	uint8_t GetVersion();
	uint8_t GetFeatures();

	uint8_t GetAddressH();
	uint8_t GetAddressL();
	uint8_t GetAirDataRate();
	uint8_t GetUARTBaudRate();
	uint8_t GetChannel();
	uint8_t GetParityBit();
	uint8_t GetTransmissionMode();
	uint8_t GetPullupMode();
	uint8_t GetWORTIming();
	uint8_t GetFECMode();
	uint8_t GetTransmitPower();

	uint8_t GetOptions();
	uint8_t GetSpeed();
		
	// methods to get data from sending unit
	uint8_t GetByte();
	bool GetStruct(const void *TheStructure, uint16_t size_);
	
	// method to send to data to receiving unit
	void SendByte(uint8_t TheByte);
	bool SendStruct(const void *TheStructure, uint16_t size_);
	
	// mehod to print parameters
	void PrintParameters();
	
	// parameters are set above but NOT saved, here's how you save parameters
	// notion here is you can set several but save once as opposed to saving on each parameter change
	// you can save permanently (retained at start up, or temp which is ideal for dynamically changing the address or frequency
	void SaveParameters(uint8_t val = PERMANENT);
	
	// MFG is not clear on what Reset does, but my testing indicates it clears buffer
	// I use this when needing to restart the EBYTE after programming while data is still streaming in
	// it does NOT return the ebyte back to factory defaults
	void Reset();

protected:

	// function to read modules parameters
	bool ReadParameters();

	// method to let method know of module is busy doing something (timeout provided to avoid lockups)
	void CompleteTask(unsigned long timeout = 0);
	
	// utility funciton to build the "speed byte" which is a collection of a few different parameters
	void BuildSpeedByte();

	// utility funciton to build the "options byte" which is a collection of a few different parameters
	void BuildOptionByte();
	
private:

	bool ReadModelData();
	void ClearBuffer();
	// variable for the serial stream
	Stream*  _s;
	Stream*  _TD;

	// pin variables
	int8_t _M0;
	int8_t _M1;
	int8_t _AUX;

	// variable for the 6 bytes that are sent to the module to program it
	// or bytes received to indicate modules programmed settings
	uint8_t _Params[6];

	// indicidual variables for each of the 6 bytes
	// _Params could be used as the main variable storage, but since some bytes
	// are a collection of several options, let's just make storage consistent
	// also Param[1] is different data depending on the _Save variable
	uint8_t _Save;
	uint8_t _AddressHigh;
	uint8_t _AddressLow;
	uint8_t _Speed;
	uint8_t _Channel;
	uint8_t _Options;
	uint8_t _Attempts;
	
	// individual variables for all the options
	uint8_t _ParityBit;
	uint8_t _UARTDataRate;
	uint8_t _AirDataRate;
	uint8_t _OptionTrans;
	uint8_t _OptionPullup;
	uint8_t _OptionWakeup;
	uint8_t _OptionFEC;
	uint8_t _OptionPower;
	uint16_t _Address;
	uint8_t _Model;
	uint8_t _Version;
	uint8_t _Features;
	uint8_t _buf;

};

/*==============================================================================================================*/

/*
create the transciever object
*/

EBYTE::EBYTE(Stream *s, uint8_t PIN_M0, uint8_t PIN_M1, uint8_t PIN_AUX)
{
	_s = s;
	_M0 = PIN_M0;
	_M1 = PIN_M1;
	_AUX = PIN_AUX;		
}

/*
Initialize the unit--basicall this reads the modules parameters and stores the parameters
for potential future module programming
*/

bool EBYTE::init(uint8_t _Attempts) {

	bool ok = true;
	
	pinMode(_AUX, INPUT);
	pinMode(_M0, OUTPUT);
	pinMode(_M1, OUTPUT);

	delay(10);
	
	if (_Attempts < 1){
		_Attempts = 1;
	}
	if (_Attempts > 10){
		_Attempts = 10;
	}
	
	SetMode(MODE_NORMAL);

	// first get the module data (must be called first for some odd reason
	
	ok = ReadModelData();

	if (!ok) {
		return false;
	}
	// now get parameters to put unit defaults into the class variables

	ok = ReadParameters();
	if (!ok) {
		return false;
	}

	return true;
}

/*
Method to indicate availability
*/

bool EBYTE::available() {

	return _s->available();
	
}

/*
Method to indicate availability
*/

void EBYTE::flush() {

	_s->flush();
	
}

/*
Method to write a single byte...not sure how useful this really is. If you need to send 
more that one byte, put the data into a data structure and send it in a big chunk
*/

void EBYTE::SendByte( uint8_t TheByte) {

	_s->write(TheByte);
	
}


/*
Method to get a single byte...not sure how useful this really is. If you need to get 
more that one byte, put the data into a data structure and send/receive it in a big chunk
*/

uint8_t EBYTE::GetByte() {

	return _s->read();

}


/*
Method to send a chunk of data provided data is in a struct--my personal favorite as you 
need not parse or worry about sprintf() inability to handle floats
TTP: put your structure definition into a .h file and include in both the sender and reciever
sketches
NOTE: of your sender and receiver MCU's are different (Teensy and Arduino) caution on the data
types each handle ints floats differently
*/

bool EBYTE::SendStruct(const void *TheStructure, uint16_t size_) {


		_buf = _s->write((uint8_t *) TheStructure, size_);
		
		CompleteTask(1000);
		
		return (_buf == size_);

}


/*
Method to get a chunk of data provided data is in a struct--my personal favorite as you 
need not parse or worry about sprintf() inability to handle floats
TTP: put your structure definition into a .h file and include in both the sender and reciever
sketches
NOTE: of your sender and receiver MCU's are different (Teensy and Arduino) caution on the data
types each handle ints floats differently
*/


bool EBYTE::GetStruct(const void *TheStructure, uint16_t size_) {
	
	_buf = _s->readBytes((uint8_t *) TheStructure, size_);

	CompleteTask(1000);

	return (_buf == size_);
}


/*
Utility method to wait until module is doen tranmitting
a timeout is provided to avoid an infinite loop
*/

void EBYTE::CompleteTask(unsigned long timeout) {

	unsigned long t = millis();

	// make darn sure millis() is not about to reach max data type limit and start over
	if (((unsigned long) (t + timeout)) == 0){
		t = 0;
	}

	// if AUX pin was supplied and look for HIGH state
	// note you can omit using AUX if no pins are available, but you will have to use delay() to let module finish
	
	// per data sheet control after aux goes high is 2ms so delay for at least that long
	// some MCU are slow so give 50 ms
	
	if (_AUX != -1) {
		
		while (digitalRead(_AUX) == LOW) {
			delay(PIN_RECOVER);
			if ((millis() - t) > timeout){
				break;
			}
		}
	}
	else {
		// if you can't use aux pin, use 4K7 pullup with Arduino
		// you may need to adjust this value if transmissions fail
		delay(1000);

	}

	// delay(PIN_RECOVER);
}

/*
method to set the mode (program, normal, etc.)
*/

void EBYTE::SetMode(uint8_t mode) {
	
	// data sheet claims module needs some extra time after mode setting (2ms)
	// most of my projects uses 10 ms, but 40ms is safer

	delay(PIN_RECOVER);
	
	if (mode == MODE_NORMAL) {

		digitalWrite(_M0, LOW);
		digitalWrite(_M1, LOW);
	}
	else if (mode == MODE_WAKEUP) {

		digitalWrite(_M0, HIGH);
		digitalWrite(_M1, LOW);

	}
	else if (mode == MODE_POWERDOWN) {
		digitalWrite(_M0, LOW);
		digitalWrite(_M1, HIGH);
	}
	else if (mode == MODE_PROGRAM) {
		digitalWrite(_M0, HIGH);
		digitalWrite(_M1, HIGH);
	}

	// data sheet says 2ms later control is returned, let's give just a bit more time
	// these modules can take time to activate pins
	delay(PIN_RECOVER);

	// clear out any junk
	// added rev 5
	// i've had some issues where after programming, the returned model is 0, and all settings appear to be corrupt
	// i imagine the issue is due to the internal buffer full of junk, hence clearing
	// Reset() *MAY* work but this seems better.
	ClearBuffer();

	// wait until aux pin goes back low
	CompleteTask(4000);
	
}


// i've asked EBYTE what's supposed to happen--got an unclear answer
// but my testing indicates it clears buffer
// I use this when needing to restart the EBYTE after programming while data is still streaming in
// to let the unit start reading from a cleared internal buffer

// it does NOT return the ebyte back to factory defaults
// if your unit gets corrupt or you need to restore values, you will have to do brute force
// example for and E44-915
// look at the data sheet for default values
//  Trans.SetAddressH(0);
//  Trans.SetAddressL(0);
//  Trans.SetSpeed(0b00011100);  
//  Trans.SetChannel(1);
//  Trans.SetOptions(0b01000100);
//  Trans.SaveParameters(PERMANENT);


void EBYTE::Reset() {

	
	SetMode(MODE_PROGRAM);

	_s->write(0xC4);
	_s->write(0xC4);
	_s->write(0xC4);

	CompleteTask(4000);

	SetMode(MODE_NORMAL);

}


void EBYTE::SetSpeed(uint8_t val) {
	_Speed = val;
}
void EBYTE::SetOptions(uint8_t val) {
	_Options = val;
}
uint8_t EBYTE::GetSpeed() {
	return _Speed ;
}
uint8_t EBYTE::GetOptions() {
	return _Options;
}

/*
method to set the high bit of the address
*/

void EBYTE::SetAddressH(uint8_t val) {
	_AddressHigh = val;
}

uint8_t EBYTE::GetAddressH() {
	return _AddressHigh;
}

/*
method to set the lo bit of the address
*/

void EBYTE::SetAddressL(uint8_t val) {
	_AddressLow = val;
}


uint8_t EBYTE::GetAddressL() {
	return _AddressLow;
}


/*
method to set the channel
*/

void EBYTE::SetChannel(uint8_t val) {
	_Channel = val;
}
uint8_t EBYTE::GetChannel() {
	return _Channel;
}


/*
method to set the air data rate
*/

void EBYTE::SetAirDataRate(uint8_t val) {

	_AirDataRate = val;
	BuildSpeedByte();
	
}

uint8_t EBYTE::GetAirDataRate() {
	return _AirDataRate;
}


/*
method to set the parity bit
*/


void EBYTE::SetParityBit(uint8_t val) {
	_ParityBit = val;
	BuildSpeedByte();
}
uint8_t EBYTE::GetParityBit( ) {
	return _ParityBit;
}

/*
method to set the options bits
*/

void EBYTE::SetTransmissionMode(uint8_t val) {
	_OptionTrans = val;
	BuildOptionByte();
}
uint8_t EBYTE::GetTransmissionMode( ) {
	return _OptionTrans;
}

void EBYTE::SetPullupMode(uint8_t val) {
	_OptionPullup = val;
	BuildOptionByte();
}
uint8_t EBYTE::GetPullupMode( ) {
	return _OptionPullup;
}

void EBYTE::SetWORTIming(uint8_t val) {
	_OptionWakeup = val;
	BuildOptionByte();
}
uint8_t EBYTE::GetWORTIming() {
	return _OptionWakeup;
}

void EBYTE::SetFECMode(uint8_t val) {
	_OptionFEC = val;
	BuildOptionByte();
}
uint8_t EBYTE::GetFECMode( ) {
	return _OptionFEC;
}

void EBYTE::SetTransmitPower(uint8_t val) {

	_OptionPower = val;
	BuildOptionByte();

}

uint8_t EBYTE::GetTransmitPower() {
	return _OptionPower;
}


/*
method to compute the address based on high and low bits
*/

void EBYTE::SetAddress(uint16_t Val) {
	_AddressHigh = ((Val & 0xFFFF) >> 8);
	_AddressLow = (Val & 0xFF);
}


/*
method to get the address which is a collection of hi and lo bytes
*/


uint16_t EBYTE::GetAddress() {
	return (_AddressHigh << 8) | (_AddressLow );
}

/*
set the UART baud rate
*/

void EBYTE::SetUARTBaudRate(uint8_t val) {
	_UARTDataRate = val;
	BuildSpeedByte();
}

uint8_t EBYTE::GetUARTBaudRate() {
	return _UARTDataRate;
}

/*
method to build the byte for programming (notice it's a collection of a few variables)
*/
void EBYTE::BuildSpeedByte() {
	_Speed = 0;
	_Speed = ((_ParityBit & 0xFF) << 6) | ((_UARTDataRate & 0xFF) << 3) | (_AirDataRate & 0xFF);
}


/*
method to build the option byte for programming (notice it's a collection of a few variables)
*/

void EBYTE::BuildOptionByte() {
	_Options = 0;
	_Options = ((_OptionTrans & 0xFF) << 7) | ((_OptionPullup & 0xFF) << 6) | ((_OptionWakeup & 0xFF) << 3) | ((_OptionFEC & 0xFF) << 2) | (_OptionPower&0b11);
}


bool EBYTE::GetAux() {
	return digitalRead(_AUX);
}


/*
method to save parameters to the module
*/

void EBYTE::SaveParameters(uint8_t val) {
	
	SetMode(MODE_PROGRAM);
	
	// ClearBuffer();

	/*
	Serial.print("val: ");
	Serial.println(val);

	Serial.print("_AddressHigh: ");
	Serial.println(_AddressHigh);

	Serial.print("_AddressLow: ");
	Serial.println(_AddressLow);

	Serial.print("_Speed: ");
	Serial.println(_Speed);

	Serial.print("_Channel: ");
	Serial.println(_Channel);

	Serial.print("_Options: ");
	Serial.println(_Options);
	*/


	_s->write(val);
	_s->write(_AddressHigh);
	_s->write(_AddressLow);
	_s->write(_Speed);
	_s->write(_Channel);
	_s->write(_Options);

	delay(PIN_RECOVER);

	CompleteTask(4000);
	
	SetMode(MODE_NORMAL);

	
}

/*
method to print parameters, this can be called anytime after init(), because init gets parameters
and any method updates the variables
*/

void EBYTE::PrintParameters() {

	_ParityBit = (_Speed & 0XC0) >> 6;
	_UARTDataRate = (_Speed & 0X38) >> 3;
	_AirDataRate = _Speed & 0X07;

	_OptionTrans = (_Options & 0X80) >> 7;
	_OptionPullup = (_Options & 0X40) >> 6;
	_OptionWakeup = (_Options & 0X38) >> 3;
	_OptionFEC = (_Options & 0X07) >> 2;
	_OptionPower = (_Options & 0X03);

	Serial.println("----------------------------------------");
	Serial.print(F("Model no.: "));  Serial.println(_Model, HEX);
	Serial.print(F("Version  : "));  Serial.println(_Version, HEX);
	Serial.print(F("Features : "));  Serial.println(_Features, HEX);
	Serial.println(F(" "));
	Serial.print(F("Mode (HEX/DEC/BIN): "));  Serial.print(_Save, HEX); Serial.print(F("/"));  Serial.print(_Save, DEC); Serial.print(F("/"));  Serial.println(_Save, BIN);
	Serial.print(F("AddH (HEX/DEC/BIN): "));  Serial.print(_AddressHigh, HEX); Serial.print(F("/")); Serial.print(_AddressHigh, DEC); Serial.print(F("/"));  Serial.println(_AddressHigh, BIN);
	Serial.print(F("AddL (HEX/DEC/BIN): "));  Serial.print(_AddressLow, HEX); Serial.print(F("/")); Serial.print(_AddressLow, DEC); Serial.print(F("/"));  Serial.println(_AddressLow, BIN);
	Serial.print(F("Sped (HEX/DEC/BIN): "));  Serial.print(_Speed, HEX); Serial.print(F("/")); Serial.print(_Speed, DEC); Serial.print(F("/"));  Serial.println(_Speed, BIN);
	Serial.print(F("Chan (HEX/DEC/BIN): "));  Serial.print(_Channel, HEX); Serial.print(F("/")); Serial.print(_Channel, DEC); Serial.print(F("/"));  Serial.println(_Channel, BIN);
	Serial.print(F("Optn (HEX/DEC/BIN): "));  Serial.print(_Options, HEX); Serial.print(F("/")); Serial.print(_Options, DEC); Serial.print(F("/"));  Serial.println(_Options, BIN);
	Serial.print(F("Addr (HEX/DEC/BIN): "));  Serial.print(GetAddress(), HEX); Serial.print(F("/")); Serial.print(GetAddress(), DEC); Serial.print(F("/"));  Serial.println(GetAddress(), BIN);
	Serial.println(F(" "));
	Serial.print(F("SpeedParityBit (HEX/DEC/BIN)    : "));  Serial.print(_ParityBit, HEX); Serial.print(F("/"));  Serial.print(_ParityBit, DEC); Serial.print(F("/"));  Serial.println(_ParityBit, BIN);
	Serial.print(F("SpeedUARTDataRate (HEX/DEC/BIN) : "));  Serial.print(_UARTDataRate, HEX); Serial.print(F("/"));  Serial.print(_UARTDataRate, DEC); Serial.print(F("/"));  Serial.println(_UARTDataRate, BIN);
	Serial.print(F("SpeedAirDataRate (HEX/DEC/BIN)  : "));  Serial.print(_AirDataRate, HEX); Serial.print(F("/"));  Serial.print(_AirDataRate, DEC); Serial.print(F("/"));  Serial.println(_AirDataRate, BIN);
	Serial.print(F("OptionTrans (HEX/DEC/BIN)       : "));  Serial.print(_OptionTrans, HEX); Serial.print(F("/"));  Serial.print(_OptionTrans, DEC); Serial.print(F("/"));  Serial.println(_OptionTrans, BIN);
	Serial.print(F("OptionPullup (HEX/DEC/BIN)      : "));  Serial.print(_OptionPullup, HEX); Serial.print(F("/"));  Serial.print(_OptionPullup, DEC); Serial.print(F("/"));  Serial.println(_OptionPullup, BIN);
	Serial.print(F("OptionWakeup (HEX/DEC/BIN)      : "));  Serial.print(_OptionWakeup, HEX); Serial.print(F("/"));  Serial.print(_OptionWakeup, DEC); Serial.print(F("/"));  Serial.println(_OptionWakeup, BIN);
	Serial.print(F("OptionFEC (HEX/DEC/BIN)         : "));  Serial.print(_OptionFEC, HEX); Serial.print(F("/"));  Serial.print(_OptionFEC, DEC); Serial.print(F("/"));  Serial.println(_OptionFEC, BIN);
	Serial.print(F("OptionPower (HEX/DEC/BIN)       : "));  Serial.print(_OptionPower, HEX); Serial.print(F("/"));  Serial.print(_OptionPower, DEC); Serial.print(F("/"));  Serial.println(_OptionPower, BIN);

	Serial.println("----------------------------------------");

}

/*
method to read parameters, 
*/

bool EBYTE::ReadParameters() {

	_Params[0] = 0;
	_Params[1] = 0;
	_Params[2] = 0;
	_Params[3] = 0;
	_Params[4] = 0;
	_Params[5] = 0;

	SetMode(MODE_PROGRAM);

	_s->write(0xC1);

	_s->write(0xC1);

	_s->write(0xC1);

	_s->readBytes((uint8_t*)&_Params, (uint8_t) sizeof(_Params));

	_Save = _Params[0];
	_AddressHigh = _Params[1];
	_AddressLow = _Params[2];
	_Speed = _Params[3];
	_Channel = _Params[4];
	_Options = _Params[5];

	_Address =  (_AddressHigh << 8) | (_AddressLow);
	_ParityBit = (_Speed & 0XC0) >> 6;
	_UARTDataRate = (_Speed & 0X38) >> 3;
	_AirDataRate = _Speed & 0X07;

	_OptionTrans = (_Options & 0X80) >> 7;
	_OptionPullup = (_Options & 0X40) >> 6;
	_OptionWakeup = (_Options & 0X38) >> 3;
	_OptionFEC = (_Options & 0X07) >> 2;
	_OptionPower = (_Options & 0X03);
	
	SetMode(MODE_NORMAL);

	if (0xC0 != _Params[0]){
		
		return false;
	}

	return true;
	
}


bool EBYTE::ReadModelData() {

	_Params[0] = 0;
	_Params[1] = 0;
	_Params[2] = 0;
	_Params[3] = 0;
	_Params[4] = 0;
	_Params[5] = 0;

	bool found = false;
	int i = 0;
	
	SetMode(MODE_PROGRAM);
	
	_s->write(0xC3);
	_s->write(0xC3);
	_s->write(0xC3);
	_s->readBytes((uint8_t*)& _Params, (uint8_t) sizeof(_Params));
	
	_Save = _Params[0];	
	_Model = _Params[1];
	_Version = _Params[2];
	_Features = _Params[3];	

	if (0xC3 != _Params[0]) {

		// i'm not terribly sure this is the best way to retry
		// may need to set the mode back to normal first....
		for (i = 0; i < 5; i++){
			// Serial.print("trying: "); Serial.println(i);
			_Params[0] = 0;
			_Params[1] = 0;
			_Params[2] = 0;
			_Params[3] = 0;
			_Params[4] = 0;
			_Params[5] = 0;

			_s->write(0xC3);
			_s->write(0xC3);
			_s->write(0xC3);
			
			_s->readBytes((uint8_t*)& _Params, (uint8_t) sizeof(_Params));
			
			/*
			Serial.print("_Attempts ");Serial.println(_Attempts);
			Serial.print("_Params[0] ");Serial.println(_Params[0]);
			Serial.print("_Params[1] ");Serial.println(_Params[1]);
			Serial.print("_Params[2] ");Serial.println(_Params[2]);
			Serial.print("_Params[3] ");Serial.println(_Params[3]);
			Serial.print("_Params[4] ");Serial.println(_Params[4]);
			Serial.print("_Params[5] ");Serial.println(_Params[5]);
			*/
			
			if (0xC3 == _Params[0]){
				found = true;
				break;
			}
			
			delay(100);
		}
	}
	else {
		found = true;
	}

	SetMode(MODE_NORMAL);

	return found;
	
}

/*
method to get module model and E50-TTL-100 will return 50
*/

uint8_t EBYTE::GetModel() {

	return _Model;
	
}

/*
method to get module version (undocumented as to the value)
*/

uint8_t EBYTE::GetVersion() {

	return _Version;
	
}

/*
method to get module version (undocumented as to the value)
*/


uint8_t EBYTE::GetFeatures() {

	return _Features;

}

/*
method to clear the serial buffer

without clearing the buffer, i find getting the parameters very unreliable after programming.
i suspect stuff in the buffer affects rogramming 
hence, let's clean it out
this is called as part of the setmode

*/
void EBYTE::ClearBuffer(){

	unsigned long amt = millis();

	while(_s->available()) {
		_s->read();
		if ((millis() - amt) > 5000) {
          Serial.println("runaway");
          break;
        }
	}

}

#endif