/*
 * DriveRobotTest.c
 *
 * Created: 11/2/2017 2:15:35 PM
 * Author : Liam
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// function prototypes
void moveMotors(uint8_t portd6, uint8_t portd5, uint8_t portd3, uint8_t portb3, uint8_t pwmspeed);
void pwmOn(void);
void pwmOff(void);
void forward(uint8_t spd);
void backward(uint8_t spd);
void turnLeft(uint8_t spd);
void turnRight(uint8_t spd);
void brake(void);
void configPWM(void);

void uartInit(void);

#define SPEED 0x30

#define UART_BAUD_RATE      9600
#define F_CPU 8000000UL
#define DIVISOR 8
#define BRR (((F_CPU/DIVISOR) / UART_BAUD_RATE) - 1)
#define PACKET_LENGTH 4

const uint8_t packet_start = 0x62;
const uint8_t packet_end = 0xe1;
static volatile uint8_t rx_data[PACKET_LENGTH];
static volatile uint8_t x;
static volatile uint8_t y;
static volatile uint8_t length = 0;


ISR(BADISR_vect) {
	return;
}

ISR (USART_RX_vect) {
	uint8_t read = UDR0;

	switch(length) {
		case 0:
			if(read == packet_start) {
				++length;
			}
			break;
		case 1:
			x = read;
			++length;
			break;
		case 2:
			y = read;
			++length;
			break;
		case 3:
			if(read == packet_end) {
				length = 0;
				forward(x);
				_delay_ms(5000);
			}
			break;
		default:
			length = 0;
			break;
	}
	
}

int main(void)
{
	int test = 0;
	
	configPWM();
	pwmOn();
	uartInit();
	sei();
	
	while(1) {
		++test;
	}
	
	return 0;
	
}

void uartInit(void) {
	UBRR0H = (unsigned char)(BRR >> 8);
	UBRR0L = (unsigned char)(BRR);
	
	UCSR0B = (1 << RXEN0) | (1 << RXCIE0); // only need RX
	UCSR0C = (0 << USBS0) | (3 << UCSZ00);
}

void moveMotors(uint8_t portd6, uint8_t portd5, uint8_t portd3, uint8_t portb3, uint8_t pwmspeed) {
	if(portb3) { // turn on portb and port d vals
		OCR2A = 0;
	}
	else {
		OCR2A = pwmspeed;
	}
	if(portd6) {
		OCR0A = 0;
	}
	else {
		OCR0A = pwmspeed;
	}
	if(portd5) {
		OCR0B = 0;
	}
	else {
		OCR0B = pwmspeed;
	}
	if(portd3) {
		OCR2B = 0;
	}
	else {
		OCR2B = pwmspeed;
	}
}

void pwmOn(void) {
	uint8_t val = 0b11110011; // set Fast PWM, non inverting
	TCCR0A = val;
	TCCR2A = val;
}

void pwmOff(void) {
	TCCR0A = 0; // disconnect oc
	TCCR2A = 0;
}

void forward(uint8_t spd) {
	//pd6, pd5, pd3, pb3
	// OCR0B is pd5, OCR0A is pd6, OCR2A is pb3, OCR2B is pd3
	moveMotors(1, 0, 0, 1, spd);
}

void turnRight(uint8_t spd) {
	moveMotors(1,0,1,0, spd);
}

void turnLeft(uint8_t spd) {
	moveMotors(0,1,0,1, spd);
}

void backward(uint8_t spd) {
	moveMotors(0,1,1,0, spd);
}

void brake(void) {
	forward(0);
	PORTD = 0b01101000;
	PORTB = 0b00001000;
}

void configPWM(void) {
	DDRB |= 0b00001000;
	DDRD |= 0b01101000;
	TCCR0B = 0b00000001;
	TCCR2B = 0b00000001;
	TIMSK0 = 0b00000110;
	TIMSK2 = 0b00000110;
}

