#include "pin.h"

Pin::Pin(
	volatile uint8_t* _portAddr,
	volatile uint8_t* _ddrAddr,
	volatile uint8_t* _pinAddr,
	uint8_t _pinNo,
	uint8_t _inverted ){
		portAddr = _portAddr;
		ddrAddr = _ddrAddr;
		pinAddr = _pinAddr;
		pinNo = _pinNo;
		inverted = _inverted;
}

void Pin::output() { *ddrAddr |= (1<<pinNo); }
void Pin::input() {	*ddrAddr &= ~(1<<pinNo); }
void Pin::write(uint8_t v) {
	v = this->inverted ? !v : v; // flips
	if(v) *portAddr |= 1 << pinNo;
	else *portAddr &= ~( 1 << pinNo);
}
uint8_t Pin::read() {
	uint8_t bit = (*pinAddr >> pinNo) & 1;
	return this->inverted ? !bit : bit;
}
