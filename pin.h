#ifndef __PIN_H_INCLUDED__
#define __PIN_H_INCLUDED__

// for uint_t types
#include <stdint.h>

class Pin
{
private:
	uint8_t pinNo;
	volatile uint8_t* portAddr;
	volatile uint8_t* ddrAddr;
	volatile uint8_t* pinAddr;
	uint8_t inverted;

public:
	// constructor
	Pin(
		volatile uint8_t* _portAddr,
		volatile uint8_t* _ddrAddr,
		volatile uint8_t* _pinAddr,
		uint8_t _pinNo,
		uint8_t _inverted );

	void output();
	void input();
	void write(uint8_t);
	uint8_t read();
};

#endif // __PIN_H_INCLUDED__
