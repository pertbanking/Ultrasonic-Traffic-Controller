/*
 * nrf.h
 *
 *  Created on: Nov 10, 2017
 *      Author: esdev
 */

#ifndef NRF_H_
#define NRF_H_

#include "NRF-Consts.h"
#include "libsoc_gpio.h"
#include "libsoc_spi.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

const uint8_t PIPE_SIZE = 3;
const uint8_t DATA_SIZE = 3;
#define SPI_DEVICE 1
#define CHIP_SELECT 0 // GPIO_123 p9_28
#define SPI_CLOCK 1000000
#define CONFIG_BYTE 0b00001010
#define READ_MODE 0x01
#define WRITE_MODE 0x00
#define NRF_CE 115 // GPIO_30 P11
#define IRQ 7


void *receiveCb;
gpio *irq, *nrf_ce;
int hasCallback = 0;

void printReg(spi* spi_dev, uint8_t addr) {
	uint8_t test[2];
	test[0] = (addr & REGISTER_MASK) | R_MASK;
	test[1] = 0xff;
	uint8_t dat[2];
	libsoc_spi_rw(spi_dev, test, dat, 2);
	printf("reg @ %d is %d\n", addr, dat[1]);
}

void resetIRQ(spi* spi_dev) {
    uint8_t write[2];
    write[0] = (0x07 & REGISTER_MASK) | W_MASK;
    write[1] = 0b01110000;
    libsoc_spi_write(spi_dev, write, 2);
}

spi* initSpiBus() {
	spi* spi_dev = libsoc_spi_init(SPI_DEVICE, CHIP_SELECT);
	libsoc_spi_set_mode(spi_dev, MODE_0);

	libsoc_spi_set_speed(spi_dev, SPI_CLOCK);

	libsoc_spi_set_bits_per_word(spi_dev, BITS_8);

	return spi_dev;
}

// only reads PIPE_SIZE bytes at a time (note: buf must be PIPE_SIZE + 1)
void receiveData(spi* spi_dev, uint8_t *buf) {
	uint8_t write[1 + PIPE_SIZE];
	write[0] = R_RX_PAYLOAD;
	for(uint i = 1; i < 1 + PIPE_SIZE; ++i) {
		write[i] = 0xff; // dummy
	}
	libsoc_spi_rw(spi_dev, write, buf, 1 + PIPE_SIZE);;
}

void initRadio(spi *spi_dev, void *receive_callback) {

	receiveCb = receive_callback;
	hasCallback = (int)receive_callback;

	irq = libsoc_gpio_request(IRQ, LS_GPIO_SHARED);
    libsoc_gpio_set_direction(irq, INPUT);
	libsoc_gpio_set_edge(irq, FALLING);


	nrf_ce = libsoc_gpio_request(NRF_CE, LS_GPIO_SHARED);

    libsoc_gpio_set_direction(nrf_ce, OUTPUT);


    if(hasCallback) {
    	libsoc_gpio_callback_interrupt(irq, receive_callback, (void *)spi_dev);
    }

	uint8_t configByte = CONFIG_BYTE | READ_MODE; // initially RX
	uint8_t configWrite[2];
	configWrite[0] = (CONFIG & REGISTER_MASK) | W_MASK;
	configWrite[1] = configByte;
	libsoc_spi_write(spi_dev, configWrite, 2);

	configWrite[0] = (EN_AA & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0x01;
	libsoc_spi_write(spi_dev, configWrite, 2); // enable auto-ack on pipe 0

	configWrite[0] = (EN_RX_ADDR & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0x01; // enable pipe 0
	libsoc_spi_write(spi_dev, configWrite, 2);


	configWrite[0] = (RX_PW_P[0] & REGISTER_MASK) | W_MASK;
	configWrite[1] = PIPE_SIZE;
	libsoc_spi_write(spi_dev, configWrite, 2); // PIPE_SIZE bytes in rx payload


	configWrite[0] = (SETUP_AW & REGISTER_MASK) | W_MASK;
	configWrite[1] = 0b0000011;
	libsoc_spi_write(spi_dev, configWrite, 2);

	libsoc_gpio_set_level(nrf_ce, HIGH); // start listening

	resetIRQ(spi_dev);
}

int sendData(uint8_t *data, spi* spi_dev) {

	if(hasCallback)
	libsoc_gpio_callback_interrupt_cancel(irq);

	libsoc_gpio_set_level(nrf_ce, 0);

	uint8_t txMode[2];
	txMode[0] = (CONFIG & REGISTER_MASK) | W_MASK;
	txMode[1] = 0b01001010;

	libsoc_spi_write(spi_dev, txMode, 2);

	uint8_t myDat[1 + DATA_SIZE];
	myDat[0] = W_TX_PAYLOAD;
	for(uint8_t i = 1; i < 1 + DATA_SIZE; ++i) {
		myDat[i] = data[i-1];
	}
	libsoc_spi_write(spi_dev, myDat, 1 + DATA_SIZE);
	uint8_t sreg;

	libsoc_gpio_set_level(nrf_ce, LOW);

	usleep(20);

	//printf("sending\n");
	//printReg(spi_dev, CONFIG);

	do {

		libsoc_gpio_set_level(nrf_ce, HIGH); // pulse data

		usleep(15);

		libsoc_gpio_set_level(nrf_ce, LOW);

		while(libsoc_gpio_get_level(irq) == HIGH);

		libsoc_spi_rw(spi_dev, &GET_NRF_SREG, &sreg, 1);

		resetIRQ(spi_dev);

	}while(!(sreg & 0b00110000));

	// printf("sent");


	// check to see if IRQ was asserted due to Auto ack

	uint8_t rxMode[2];
	rxMode[0] = (CONFIG & REGISTER_MASK) | W_MASK;
	rxMode[1] = (CONFIG_BYTE | READ_MODE);

	libsoc_spi_write(spi_dev, rxMode, 2);


	libsoc_gpio_set_level(nrf_ce, 1); // lisetn again


	if(hasCallback)
	libsoc_gpio_callback_interrupt(irq, receiveCb, (void *)spi_dev);


	return !(sreg & 0b00100000); // non-zero for error
}


#endif /* NRF_H_ */
