/*
 * DriveRobotTest.c
 *
 * Created: 11/2/2017 2:15:35 PM
 * Author : Liam and Jash
 */ 

#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 8000000UL
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
void ultrasoundEmitterInit(void);

#define SPEED 0x30

#define UART_BAUD_RATE      9600
#define DIVISOR 8
#define BRR (((F_CPU/DIVISOR) / UART_BAUD_RATE) - 1)
#define PACKET_LENGTH 4

const uint8_t packet_start = 0x62;
const uint8_t packet_end = 0xe1;

static volatile uint8_t pulseEmit;
static volatile uint8_t rx_data[PACKET_LENGTH];
static volatile uint8_t x;
static volatile uint8_t y;
static volatile uint8_t length = 0;


ISR(BADISR_vect)
{
	// You sure you don't want to use this to debug??
	// - Josh
	return;
}

ISR(USART_RX_vect)
{
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

ISR(TIMER1_COMPA_vect)
{
	if (pulseEmit > 0)
	{
		if (PORTD & (1<<PORTD7))
		{
			//--pulseEmit; // comment out for debugging purposes
			PORTD &= ~(1<<PORTD7);
		}
		else
			PORTD |= 1<<PORTD7;
	}
}

int main(void)
{
	int test = 0;
	
	configPWM();
	pwmOn();
	ultrasoundEmitterInit();
	uartInit();
	sei();
	
	while(1) {
		//++test;
		pulseEmit = 1;
	}
	
	return 0;
	
}


// ************************************
// Josh added code here... read the code to see what Josh did
// ************************************

void ultrasoundEmitterInit(void)
{
	// set the direction and the initial value of the PD7 pin
	DDRD |= 1<<DDD7;
	PORTD &= ~(1<<PORTD7);
	
	// configure a timer (TIMER1) to interrupt continuously @ 40kHz
	// mode = 4: CTC, trigger ISR @ OCR1A
	TCCR1A = 0b00000000;
	TCCR1B = 0b00001000 /*|(1<<CS10)*/;
	TCCR1C = 0b00000000;
	OCR1AH = 0b00000000;
	OCR1AL = 0b11111010; // = 250
	OCR1BH = 0b11111111;
	OCR1BL = 0b11111111; // This just so the ISR bit for OCF1B doesn't get flipped.
	
	TIMSK1 |= 1<<OCIE1A;
	TIFR1 = 0b00000000;
	
	// If pulseEmit is greater than 0, then ultrasound will be emitted.
	// Otherwise, the ISR will exit immediately.
	// Set pulseEmit equal to the number of pulses you want to send over ultrasound;
	// it will decrease by 1 every time the ISR is called (given it is > 0).
	pulseEmit = 0;
	
	TCCR1B |= 1<<CS10;
}

// END Josh's code

void uartInit(void)
{
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
	TIMSK0 = 0b00000000;  // Changed these for you.
	TIMSK2 = 0b00000000;  // Interrupts aren't used in Fast PWM, so TIMSK's aren't set.
}						  // This was slowing down runtime substantially.

