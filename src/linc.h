
#ifndef __LINC_H_INCLUDED__
#define __LINC_H_INCLUDED__

#define F_CPU 16000000UL // 16 MHz
#define BAUD 9600

// for uint_t types
#include <stdint.h>
#include <stdio.h> //sprintf
#include <string.h> //array de ids
#include <stdlib.h> // malloc
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h> // para atomic
#include <util/setbaud.h> // uart
#include "pin.h"
#include "new.h"

class Linc
{
	// ponteiro de action function
	typedef void (*action_t)(Linc*);

	// estados
	enum LincState {
		STATE_BUSY, // ignore interrupts
		STATE_BREAK,
		STATE_SYNCH,
		STATE_ID,
		STATE_DATA_IN,
	};

private:
	LincState state; // Slave current state
	Pin *pin_tx;
	Pin *pin_rx;
	uint8_t is_master;
	uint32_t bittime; // tempo entre bits
	uint8_t synch_counter; // bits recebidos no modo SYNCH
	uint8_t last_id; // último id recebido
	uint8_t last_check; /// último checksum
	uint8_t data_in_counter; // data_in bytes counter
	uint8_t data_in[8]; // bytes recebidos
	action_t actions[256]; // array de funções para cada id

	// Tempo
	void resetMicros();
	uint16_t getMicros();
	void waitMicros(uint16_t);

	// Tx e Rx
	uint8_t getByte();
	void putByte(uint8_t);
	uint8_t get256sum(uint8_t* check);
	void sendBreak();
	void sendSynch();


public:
	// construtor
	Linc(Pin*, Pin*, uint8_t);
	uint8_t isMaster();
	void sendHeader(uint8_t);
	void setIdAction (uint8_t, action_t);
	void sendResponse(uint8_t*);
	uint8_t* getResponse();
	uint8_t isValidResponse();


	// interrupção RX
	void interruptHandler();

	// USART
	void uartInit();
	void uartPutString(char*);
	char uartGetchar();
};

#endif // __LINC_H_INCLUDED__
