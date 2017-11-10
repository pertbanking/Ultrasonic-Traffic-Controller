#include "libsoc_spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "NRF-Consts.h"
#include "libsoc_gpio.h"
// #include "libsoc_debug.h"


uint8_t receiveData(spi* spi_dev);
int sendData(uint8_t data, spi* spi_dev);
void initRadio(spi* spi_dev);
spi* initSpi(void);
void resetIRQ(spi * spi_dev);
void handleData(spi *spi_dev, uint8_t data);
void updateTimeSpec(struct timespec *ts, int nsPeriod);
void configureTX(spi *spi_dev);

#define SPI_DEVICE 1
#define CHIP_SELECT 0 // GPIO_123 p9_28
#define SPI_CLOCK 1000000
#define CONFIG_BYTE 0b00001010
#define READ_MODE 0x01
#define WRITE_MODE 0x00
#define NRF_CE 49 // GPIO_49 p9_23
#define IRQ 7

#define TIMEOUT 0b00010000

gpio *irq, *nrf_ce;

// spi pins: sclk = p9_31 gpio110
// DI (MOSI) = p9_29 gpio111
// DO (MISO) = p9_30 gpio112

int receive_callback(void *arg) {
	spi* spi_dev = (spi*)arg;
	uint8_t sreg;
	printf("irq received");
	libsoc_gpio_set_level(nrf_ce, LOW);

	if (libsoc_gpio_get_level(irq) != HIGH) {
		// printf("irq!");
		libsoc_spi_rw(spi_dev, &GET_SREG, &sreg, 1);
		if(sreg & 0b01000000) {
			printf("match!\n");
			handleData(spi_dev, receiveData(spi_dev));
		}
	}
	libsoc_gpio_set_level(nrf_ce, HIGH); // listen again
	resetIRQ(spi_dev);

	return 0;
}

int main (void) {
	spi* spi_dev = initSpi();
	initRadio(spi_dev);
	// configureTX(spi_dev);
	uint8_t read[2];
	uint8_t cWrite[2];

	printf("1\n");

	cWrite[0] = (REGISTER_MASK & CONFIG) | R_MASK;
	cWrite[1] = 0xff; // dummy byte
	// 0b00011011
	libsoc_spi_rw(spi_dev, cWrite, read, 2);

	printf("2\n");

	printf("reg val: %d\n", read[1]);

	printf("3??\n");
	while(1) {
		// uint8_t data = 0xcf;
		// libsoc_spi_write(spi_dev, &data, 1);
		char c[2];
		scanf("%c", c);
		if(c[0] == 'f') {
			printf("sending forward\n");
			printf("result: %d\n", sendData(0xcf, spi_dev));
		} else if(c[0] == 's') {
			printf("sending stop\n");
			printf("result: %d\n", sendData(0x00, spi_dev));
		}
	}

	printf("done\n");
	return 0;
}

// only reads one byte at a time
uint8_t receiveData(spi* spi_dev) {
	uint8_t write[2];
	write[0] = R_RX_PAYLOAD;
	write[1] = 0xff; // dummy
	uint8_t ret[2];
	libsoc_spi_rw(spi_dev, write, ret, 2);
	return ret[1];
}

spi* initSpi(void) {

	// libsoc_set_debug(1);
	spi* spi_dev = libsoc_spi_init(SPI_DEVICE, CHIP_SELECT);
	libsoc_spi_set_mode(spi_dev, MODE_0);

	libsoc_spi_set_speed(spi_dev, SPI_CLOCK);

	libsoc_spi_set_bits_per_word(spi_dev, BITS_8);

	// libsoc_set_write_enable(spi_dev); from spi_test example
	// may or may not be necessary

	return spi_dev;
}
void initRadio(spi *spi_dev) {
	irq = libsoc_gpio_request(IRQ, LS_GPIO_SHARED);
    libsoc_gpio_set_direction(irq, INPUT);
	libsoc_gpio_set_edge(irq, FALLING);


	nrf_ce = libsoc_gpio_request(NRF_CE, LS_GPIO_SHARED);

    libsoc_gpio_set_direction(nrf_ce, OUTPUT);



	libsoc_gpio_callback_interrupt(irq, &receive_callback, (void *)spi_dev);


	uint8_t configByte = CONFIG_BYTE | READ_MODE; // initially RX
	uint8_t configWrite[2];
	configWrite[0] = (CONFIG & REGISTER_MASK) | W_MASK;
	configWrite[1] = configByte;
	libsoc_spi_write(spi_dev, configWrite, 2);

	configWrite[0] = (EN_AA & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b00000001;
	libsoc_spi_write(spi_dev, configWrite, 2); // enable auto-acknowledgement on pipe 0

	configWrite[0] = (EN_RX_ADDR & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b00000001; // enable data pipe 0
	libsoc_spi_write(spi_dev, configWrite, 2);

	configWrite[0] = (RX_PW_P[0] & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b00000001;
	libsoc_spi_write(spi_dev, configWrite, 2); // 1 byte in RX payload in data pipe 0

	configWrite[0] = (SETUP_AW & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b0000011;
	libsoc_spi_write(spi_dev, configWrite, 2);

	libsoc_gpio_set_level(nrf_ce, HIGH); // start listening

	resetIRQ(spi_dev);
}

int sendData(uint8_t data, spi* spi_dev) {
	libsoc_gpio_callback_interrupt_cancel(irq);

	libsoc_gpio_set_level(nrf_ce, 0);
	// usleep(500);

	uint8_t txMode[2];
	txMode[0] = (CONFIG & REGISTER_MASK) | W_MASK;
	txMode[1] = 0b01001010;

	libsoc_spi_write(spi_dev, txMode, 2);

	/*
	uint8_t read[2];
	uint8_t cWrite[2];


	cWrite[0] = (REGISTER_MASK & CONFIG) | R_MASK;
	cWrite[1] = 0xff; // dummy byte
	// 0b00011011
	libsoc_spi_rw(spi_dev, cWrite, read, 2);

	*/

	uint8_t myDat[2];
	myDat[0] = W_TX_PAYLOAD;
	myDat[1] = data;
	libsoc_spi_write(spi_dev, myDat, 2);





	uint8_t sreg;

	libsoc_gpio_set_level(nrf_ce, LOW);
	usleep(10);
	do {

		libsoc_gpio_set_level(nrf_ce, HIGH); // pulse data

		//clock_gettime(CLOCK_REALTIME, &tspec);

		usleep(15);

		libsoc_gpio_set_level(nrf_ce, LOW);

		/*
		uint8_t read[2];
		write[0] = (REGISTER_MASK & CONFIG) | R_MASK;
		write[1] = 0xff; // dummy byte
		// 0b00011011
		libsoc_spi_rw(spi_dev, write, read, 2);

		printf("reg val: %x\n", read[1]);
		*/

		// printf("low write");

		// printf("before read\n");
		while(libsoc_gpio_get_level(irq) == HIGH);
		// printf("after read\n");
		libsoc_spi_rw(spi_dev, &GET_SREG, &sreg, 1);
		resetIRQ(spi_dev);
		// printf("sreg: %x\n", sreg);

	}while(!(sreg & 0b00110000));
	// check to see if IRQ was asserted due to Auto ack

	uint8_t rxMode[2];
	rxMode[0] = (CONFIG & REGISTER_MASK) | W_MASK;
	rxMode[1] = (CONFIG_BYTE | READ_MODE);

	libsoc_spi_write(spi_dev, rxMode, 2);


	libsoc_gpio_set_level(nrf_ce, 1); // lisetn again


	libsoc_gpio_callback_interrupt(irq, &receive_callback, (void *)spi_dev);



	return !(sreg & 0b00100000); // non-zero for error
}

void resetIRQ(spi* spi_dev) {
    uint8_t write[2];
    write[0] = (0x07 & REGISTER_MASK) | W_MASK;
    write[1] = 0b01110000;
    libsoc_spi_write(spi_dev, write, 2);
}

void handleData(spi *spi_dev, uint8_t data) {
	// do somethign with data
	printf("%d\n", data);
}

void updateTimeSpec(struct timespec *ts, int nsPeriod) {
	if(ts->tv_nsec + nsPeriod > 1000000000) {
		ts->tv_sec++;
	}
	ts->tv_nsec = (ts->tv_nsec + nsPeriod)%1000000000;
}

void configureTX(spi *spi_dev) {
	irq = libsoc_gpio_request(IRQ, LS_GPIO_SHARED);
	libsoc_gpio_set_direction(irq, INPUT);
	libsoc_gpio_set_edge(irq, FALLING);


	nrf_ce = libsoc_gpio_request(NRF_CE, LS_GPIO_SHARED);

	libsoc_gpio_set_direction(nrf_ce, OUTPUT);

	libsoc_gpio_set_level(nrf_ce, LOW);
	usleep(10000);

	uint8_t configWrite[2];
	configWrite[0] = (CONFIG & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b01001010;  // config stuff. show max_rt and tx_ds
	// interrupts. mask rx_dr interrupt. enable crc, crc 1 byte,
	// power up, ptx mode
	libsoc_spi_write(spi_dev, configWrite, 2);


	configWrite[0] = (EN_AA & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b00000001;
	libsoc_spi_write(spi_dev, configWrite, 2); // enable auto-acknowledgement on pipe 0

	configWrite[0] = (EN_RX_ADDR & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b00000001; // enable data pipe 0
	libsoc_spi_write(spi_dev, configWrite, 2);

	configWrite[0] = (RX_PW_P[0] & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b00000001;
	libsoc_spi_write(spi_dev, configWrite, 2); // 1 byte in RX payload in data pipe 0

	configWrite[0] = (SETUP_AW & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b0000011;
	libsoc_spi_write(spi_dev, configWrite, 2);


}
