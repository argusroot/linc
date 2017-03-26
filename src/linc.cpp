#include "linc.h"

Linc::Linc(Pin* _pin_tx, Pin* _pin_rx, uint8_t _master) {
	// master
	this->is_master = _master;
	// inicia uart
	this->uartInit();
	// se for master inicia BUSY
	this->state = _master ? STATE_BUSY : STATE_BREAK;
	// seta os pinos
	pin_tx = _pin_tx;
	pin_rx = _pin_rx;
	pin_tx->output();
	pin_rx->input();
	// seta bittime padrão em us
	this->bittime = 50;
	// inicializa array de actions
	for(int i=0; i<256; i++)
		this->actions[i]=NULL;

}

void Linc::resetMicros(){
	TCCR0B = 0; // para timer
	TCNT0 = 0; // zera
	TCCR0B |= (1 << CS01) | (1 << CS00); // precalar 64
}

uint16_t Linc::getMicros()
{
    uint16_t micros_return;
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
		// tick de 4 us
		micros_return = ((TCNT0+1) * 4 ) + 4;
    }
    return micros_return;
}

void Linc::waitMicros(uint16_t micros){
	// se for maior que limite do contador de 8bits
	// (1/clock) * prescalar * 255bits
	uint16_t max = 1023;
	while(micros > max){
		this->waitMicros(max);
		micros -= max;
	}
	this->resetMicros();
	while( this->getMicros() < micros );
}

uint8_t Linc::getByte(){
	uint8_t byte = 0x00;
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		for(uint8_t i=0; i<8; i++){
			byte |=  this->pin_rx->read() ? (1<<i) : 0;
			this->waitMicros(bittime);
		}
	}
	return byte;
}

void Linc::putByte(uint8_t byte){
	ATOMIC_BLOCK(ATOMIC_FORCEON) {
		pin_tx->write(0); // start bit
		this->waitMicros(this->bittime);
		for(int i=0; i<8; i++){
			pin_tx->write( (byte >> i) & 1 );
			this->waitMicros(this->bittime);
		}
		pin_tx->write(1); // stop bit
		this->waitMicros(this->bittime);
	}
}

uint8_t Linc::get256sum(uint8_t *data){
	uint8_t sum = 0x00;
	for(uint8_t i=0; i<8; i++){
		sum += data[i];
	}
	return ~sum;
}

void Linc::sendBreak(){
	pin_tx->write(0); // 13 low
	this->waitMicros(this->bittime * 13);
	pin_tx->write(1); // 4 high
	this->waitMicros(this->bittime * 4);
}

void Linc::sendSynch(){
	this->putByte(0x55);
}

/*******************************************************************
	PUBLIC
*******************************************************************/

uint8_t Linc::isMaster(){
	return this->is_master;
}

void Linc::sendHeader(uint8_t _id){
	// ignore interrupts
	this->state = STATE_BUSY;
	this->last_id = _id;
	this->sendBreak();
	this->sendSynch();
	this->putByte(_id);
	this->state = STATE_BUSY;
	// se tiver action para este header espera e chama agora
	if(this->actions[_id] != NULL){
		this->actions[_id](this);
	}
}

void Linc::setIdAction(uint8_t _id, action_t _action){
	if(_id >= 0 && _id < 256 && _action != 0)
		this->actions[_id] = _action;
}

void Linc::sendResponse(uint8_t *data){
	this->state = STATE_BUSY;
	this->waitMicros(this->bittime*5);
	for(uint8_t i=0; i<8; i++){
		putByte( data[i] );
	}
	putByte( this->get256sum(data) );
	this->state = STATE_BREAK;
}

uint8_t *Linc::getResponse(){
	if(data_in_counter == 8){
		this->data_in_counter = 0;
		return data_in;
	} else {
		this->data_in_counter = 0;
		this->state = STATE_DATA_IN;
		return NULL;
	}
}

uint8_t Linc::isValidResponse(){
	uint8_t calc = this->get256sum(this->data_in);
	return  calc == this->last_check;
}

void Linc::interruptHandler() {

	// se não for um sinal baixo ou estiver ocupado, ignora interrupt
	// se for master e não estiver recebendo uma resposta, ignora também
	if( pin_rx->read() != 0 || this->state == STATE_BUSY ||
 		(isMaster() && this->state != STATE_DATA_IN) ) {
			return;
		}

	// switch de tratamento da interrupção
	switch(this->state){

		// precisa receber pelo menos 11 bits low
		case LincState::STATE_BREAK:
			this->waitMicros(bittime/2);
			uint8_t bits;
			for(bits=0; bits<11 && pin_rx->read()==0; bits++){
				waitMicros(bittime);
			}
			if(bits >= 10){
				this->state = STATE_SYNCH;
				this->synch_counter = 0;
			}
		break;

		// recebe 8 bits (5 lows) para sincronizar
		case LincState::STATE_SYNCH:
			if( this->synch_counter == 0){
				this->resetMicros();//this->micros=0;
			} else if( this->synch_counter == 4){
				// remove 3us de possível tick adicional
				this->bittime = (this->getMicros() / 8) - 3;
				this->state = STATE_ID;
				return;
			}
			this->synch_counter++;
		break;

		// recebe o ID do header
		case LincState::STATE_ID:
			this->state = STATE_BUSY;
			// espera estabilizar o bit
			this->waitMicros(bittime);
			// lê o byte
			this->last_id = this->getByte();
			this->data_in_counter = 0;
			this->state = STATE_BREAK;
			// chama action se tiver
			this->actions[this->last_id](this);
		break;

		// se alguma action habilitou a captura de mensagem
		case LincState::STATE_DATA_IN:
			this->state = STATE_BUSY;
			// espera estabilizar o bit
			this->waitMicros(bittime);
			// se for o slave do master espera um pouco mais (?)
			if(this->isMaster()) this->waitMicros(bittime);
			// lê 8 bytes e 1 byte de checksum
			if(this->data_in_counter < 8){
				this->data_in[data_in_counter++] = this->getByte();
				this->state = STATE_DATA_IN;
			} else {
				// pega checksum
				this->last_check = this->getByte();
				// chama action se tiver
				if(this->actions[this->last_id] != NULL)
					this->actions[this->last_id](this);
				// volta ao estado normal
				this->state = is_master ? STATE_BUSY : STATE_BREAK;
			}
		break;

		default:
		break;
	}
}

/*********************************************************************
	USART utils
********************************************************************/
void Linc::uartInit() {
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;
#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);   /* Enable RX and TX */
}

void Linc::uartPutString(char* c) {
	while(*c){
		loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */
	    UDR0 = *c++;
	}
}

char Linc::uartGetchar() {
	loop_until_bit_is_set(UCSR0A, RXC0); /* Wait until data exists. */
    return UDR0;
}
