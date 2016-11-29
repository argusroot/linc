
#include "linc.h"
#include <util/delay.h> // para _delay_us
#include "pin.h"

Linc *linc;

void action22(Linc* l){

	if(l->isMaster()){
		uint8_t msg2[]={0xAA,0xBB,0xCC,0xDD,0xEE,0xFF,0x99,0x88};
		linc->sendResponse(msg2);
	} else {
		char msg3[60];
		uint8_t *data = l->getResponse();
		if(data != NULL){
			if(l->isValidResponse()){
				sprintf(msg3,(char*)"\n data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
			} else {
				sprintf(msg3,(char*)"\n CHECKSUM error");
			}
			linc->uartPutString((char *)msg3);
		}
	}
}

void action33(Linc* l){
	if(!l->isMaster()){
		uint8_t msg1[]={0xff,0xcc,0xff,0xcc,0xff,0xcc,0xff,0xcc};
		linc->sendResponse(msg1);
	} else {
		uint8_t *data = l->getResponse();
		if(data != NULL){
			//l->uartPutString((char *)"\nrecebeu");
			char *msg = (char*) calloc(60,1);
			if(l->isValidResponse()){
				sprintf(msg,(char*)"\n data: 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X",
					data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7]);
			} else {
				sprintf(msg,(char*)"\n CHECKSUM error");
			}
			l->uartPutString((char *)msg);
			free(msg);
		}
	}
}

int main(void)
{
	Pin *pin_tx = new Pin(&PORTB, &DDRB, &PINB, 5, 1); // 13
	//Pin *pin_tx = new Pin(&PORTB, &DDRB, &PINB, 0, 1); // 8
	Pin *pin_rx = new Pin(&PORTD, &DDRD, &PIND, 7, 1); // 7

	cli();
	// habilita interrupção no pino digital 7
	PCICR |= (1 << PCIE2);
  	PCMSK2 |= (1 << PCINT23);
	sei();

	// instancia o Lin com
	linc = new Linc(pin_tx, pin_rx, 1);

	linc->setIdAction(0x22, action22);
	linc->setIdAction(0x33, action33);

	uint8_t interval = 50;

	// se for master
	while(1){
		if(linc->isMaster()){
			_delay_ms(500);
			linc->sendHeader(0x22);
			 _delay_ms(interval);
			linc->sendHeader(0x33);
			_delay_ms(interval);
		    linc->sendHeader(0x22);
			_delay_ms(interval);
		   linc->sendHeader(0x33);
		}
	}
}



// trata interrupções no pino de RX
ISR(PCINT2_vect) {
	linc->interruptHandler();
}
