#include <stdint.h>
#include "nrf.h"
#include "BT.h"
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <time.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>

void handleData(uint8_t *data);
void updateTimeSpec(struct timespec *ts, long nsPeriod);
void *sendPing(void *p);
void *getPhone(void *p);
void *tower(void *p);


const uint8_t SEND_PING = 20;
const uint8_t TOWER_ONE = 1;


pthread_t ping1, manual, receiveTime;
pthread_mutex_t radio_lock;
int dataAvailable = 0;
int receiving = 1;

/*
int receive_callback(void *arg) {
	spi* spi_dev = (spi*)arg;
	uint8_t sreg;

	// printf("received");

	libsoc_gpio_set_level(nrf_ce, LOW);

	if (libsoc_gpio_get_level(irq) != HIGH) {
		// printf("irq!");
		libsoc_spi_rw(spi_dev, &GET_NRF_SREG, &sreg, 1);
		if(sreg & 0b01000000) {
			dataAvailable = 1;
		}
	}
	libsoc_gpio_set_level(nrf_ce, HIGH); // listen again

	return 0;
}
*/

int main (void) {
	spi* spi_dev = initSpiBus();
	// initRadio(spi_dev, &receive_callback);
	initRadio(spi_dev, (void *)0);

	printReg(spi_dev, CONFIG);

	pthread_attr_t myattr;
    pthread_attr_init(&myattr);

    pthread_mutexattr_t mutexAttr;
    pthread_mutexattr_init(&mutexAttr);
    pthread_mutex_init(&radio_lock, &mutexAttr);
    int err;
	err = pthread_create(&ping1, &myattr, sendPing, (void *)spi_dev);
	err = pthread_create(&manual, &myattr, getPhone, (void *)spi_dev);
	// err = pthread_create(&receiveTime, &myattr, tower, (void *)spi_dev);
	pthread_attr_destroy(&myattr);

	uint8_t sreg;
	// poll for interrupt
	while(1) {
		if (libsoc_gpio_get_level(irq) != HIGH) {
						// printf("irq!");
			libsoc_spi_rw(spi_dev, &GET_NRF_SREG, &sreg, 1);
			if(sreg & 0b01000000) {
				dataAvailable = 1;
				// printf("match!\n");
				uint8_t pipe[PIPE_SIZE + 1];
				pthread_mutex_lock(&radio_lock);
				receiveData(spi_dev, pipe);
				resetIRQ(spi_dev);
				pthread_mutex_unlock(&radio_lock);
				handleData(pipe);
				dataAvailable = 0;
			}
		}
	}

	pthread_join(ping1, NULL);
	pthread_join(manual, NULL);
	// pthread_join(receiveTime, NULL);

	return 0;
}

void handleData(uint8_t *data) {
	// do somethign with data
	int time_us = (data[2] | (data[3] << 8));
	double distance_cm = time_us*0.0343;
	printf("the robot is %f cm away\n", distance_cm);
}

void updateTimeSpec(struct timespec *ts, long nsPeriod) {
	if(ts->tv_nsec + nsPeriod > 1000000000) {
		ts->tv_sec++;
	}
	ts->tv_nsec = (ts->tv_nsec + nsPeriod)%1000000000;
}

void *sendPing(void *p) {
	spi *spi_dev = (spi*)p;
	uint8_t myData[DATA_SIZE];
	myData[0] = SEND_PING;
	myData[1] = 0xff;
	myData[2] = 0xff; // dummies
	// struct timespec tspec;
	// clock_gettime(CLOCK_REALTIME, &tspec);
	// long period = 1000000000;
	for(;;) {
		pthread_mutex_lock(&radio_lock);
		receiving = 0;
		printf("result: %d\n", sendData(myData, spi_dev));
		receiving = 1;
		pthread_mutex_unlock(&radio_lock);
		printf("pinging....\n");
		usleep(250000);
		// updateTimeSpec(&tspec, period);
		// clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tspec, NULL);
	}
	return (void *)0;
}

void *tower(void *p) {
	uint8_t sreg;
	spi *spi_dev = (spi *)p;
	while(1) {
		if (libsoc_gpio_get_level(irq) != HIGH) {
				// printf("irq!");
			libsoc_spi_rw(spi_dev, &GET_NRF_SREG, &sreg, 1);
			if(sreg & 0b01000000) {
				dataAvailable = 1;
				resetIRQ(spi_dev);
				// printf("match!\n");
				uint8_t pipe[PIPE_SIZE + 1];
				pthread_mutex_lock(&radio_lock);
				receiveData(spi_dev, pipe);
				pthread_mutex_unlock(&radio_lock);
				handleData(pipe);
				dataAvailable = 0;
			}
		}
	}
}

void *getPhone(void *p) {
	spi *spi_dev = (spi *) p;
	uint8_t data[DATA_SIZE];
	data[0] = 0xcf;

	int fd;
	if((fd = open("/dev/ttyS1", O_RDONLY | O_NOCTTY|O_NDELAY)) < 0)
		{
			perror("open failed for /dev/ttyS1");
			return UART_OPEN_ERROR;
		}
	struct termios uart1;
	cfsetospeed(&uart1, B9600);

	uart1.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
	uart1.c_iflag = IGNPAR | ICRNL;
	uart1.c_oflag = 0;
	uart1.c_lflag = 0;
	tcflush(fd, TCIFLUSH);
	tcsetattr(fd, TCSANOW, &uart1);

	uint8_t receivedX = 0, receivedY = 0;
	uint8_t x, y;
	uint8_t buffer = 0;
	uint8_t received = 0;

	while(1) {
		while(read(fd, (void *)&buffer, 1) <= 0);
		// printf("got: %x from buffer\n", buffer);
		++received;
		if(receivedX) {
			x = buffer;
			receivedX = 0;
		} else if (receivedY) {
			y = buffer;
			receivedY = 0;
		}
		if(buffer == 20) {
			receivedX = 1;
		} else if (buffer == 30) {
			receivedY = 1;
		} else if (buffer == 16 && !receivedX && !receivedY) {
			data[1] = x;
			data[2] = y;
			printf("x: %d, y: %d\n", x, y);
			// pthread_mutex_lock(&radio_lock);
			printf("results: %d\n", sendData(data, spi_dev));
			// pthread_mutex_unlock(&radio_lock);
		}
	}
	close(fd);
}
