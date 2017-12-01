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
double axisDistance(double rad1, double rad2);
void getCoords(double *xy, int missingTower, double *last_tower_Dist);
int pingCoords(double *xy, spi* spi_dev, uint8_t head, double *dist);
void *driveAutonomous(void *p);
double getDist(double x1, double y1, double x2, double y2);
double rotX(int right, double curx, double cury);
double rotY(int right, double curx, double cury);

// BLUE LED ROBOT IS MANUAL
const uint8_t SEND_PING_MANUAL = 20;
const uint8_t SEND_PING_AUTONOMOUS = 21;
const uint8_t SEND_PING_TOWER = 22;
const uint8_t MANUAL_MOVE = 0xcc;
const uint8_t AUTONOMOUS_MOVE = 0xcf;
const uint8_t TOWER_ONE = 1;
const uint8_t MAX_LENGTH = 140;
const double TOWER_TO_TOWER = 110.49; // 110 cm from tower to tower


pthread_t ping1, manual, receiveTime, auton;
pthread_mutex_t radio_lock;
int dataAvailable = 0;
int receiving = 1;
int pingWait = 0;

double last_tower_dist[4]; // the last distance received
double last_tower_dist_aut[4];
int last_tower_ping[4]; // how many pings ago it last got a signal
double theory_dist[4];
double xy_manual[2];
double xy_manual_theory[2];
double xy_aut[2];
double last_xy_aut[2];
double autonomous_heading[2]; // the direction

int changeDirection = 0;

/*
 * 1----3
 * |	|
 * 2----4
 *
 *
 */

/*
int receive_callback(void *arg) {
	spi* spi_dev = (spi*)arg;
	uint8_t sreg;

	// printf("received");
	if(pingWait) {
		printf("something\n");
		pthread_mutex_lock(&radio_lock);
		libsoc_gpio_set_level(nrf_ce, LOW);

		//if (libsoc_gpio_get_level(irq) != HIGH) {
			printf("irq!\n");
			libsoc_spi_rw(spi_dev, &GET_NRF_SREG, &sreg, 1);
			if(sreg & 0b01000000) {
				dataAvailable = 1;
				resetIRQ(spi_dev);
				uint8_t pipe[PIPE_SIZE + 1];
				receiveData(spi_dev, pipe);
				pingWait = 0;
				handleData(pipe);

			}
		//}
		libsoc_gpio_set_level(nrf_ce, HIGH); // listen again
		pthread_mutex_unlock(&radio_lock);
	}

	return 0;
}
*/


int main (void) {
	spi* spi_dev = initSpiBus();
	// initRadio(spi_dev, &receive_callback);
	initRadio(spi_dev, (void *)0);

	printf("config is: %d\n",getReg(spi_dev, CONFIG));

	pthread_attr_t myattr;
    pthread_attr_init(&myattr);

    pthread_mutexattr_t mutexAttr;
    pthread_mutexattr_init(&mutexAttr);
    pthread_mutex_init(&radio_lock, &mutexAttr);
    int err;
	err = pthread_create(&ping1, &myattr, sendPing, (void *)spi_dev);
	err = pthread_create(&manual, &myattr, getPhone, (void *)spi_dev);
	err = pthread_create(&auton, &myattr, driveAutonomous, (void *)spi_dev);
	pthread_attr_destroy(&myattr);

	xy_manual[0] = 55;
	xy_manual[1] = 55;

	for(uint8_t i = 0; i < 4; ++i) {
		last_tower_ping[i] = 10; // intialize to no pings received recently
	}
	/*
	while(1) {
		// pingCoords(xy_manual, spi_dev, SEND_PING_MANUAL);
		//usleep(1000000);
	}*/
	pthread_join(auton, NULL);
	pthread_join(ping1, NULL);
	pthread_join(manual, NULL);

	return 0;
}

void *driveAutonomous(void *p) {
	spi *spi_dev = (spi *) p;
	uint8_t data[DATA_SIZE];
	data[0] = AUTONOMOUS_MOVE;

	last_xy_aut[0] = 0;
	last_xy_aut[1] = 0;

	autonomous_heading[0] = 0;
	autonomous_heading[1] = 1;

	data[1] = (uint8_t)(8 - autonomous_heading[0]*8);
	data[2] = (uint8_t)(autonomous_heading[1]*8);

	printf("dat 1: %d, dat 2: %d\n", data[1], data[2]);

	pthread_mutex_lock(&radio_lock);
	while(sendData(data, spi_dev, 3));
	pthread_mutex_unlock(&radio_lock);

	printf("sent initial direction\n");

	while(1) {
		if(changeDirection) {
			printf("valid data!\n");
			changeDirection = 0;
			double difX = xy_aut[0] - last_xy_aut[0];
			double difY = xy_aut[1] - last_xy_aut[1];
			double tmp_x = autonomous_heading[0];
			double tmp_y = autonomous_heading[1];

			double rotLeftX = rotX(0, tmp_x, tmp_y);
			double rotLeftY = rotY(0, tmp_x, tmp_y);

			double rotRightX = rotX(1, tmp_x, tmp_y);
			double rotRightY = rotY(1, tmp_x, tmp_y);

			if(getDist(xy_manual[0], xy_manual[1], xy_aut[0], xy_aut[1]) <= 900) {
				printf("near!\n");

				double distLeft = getDist((xy_aut[0] + rotLeftX), (xy_aut[1] + rotLeftY), xy_manual[0], xy_manual[1]);
				double distRight = getDist((xy_aut[0] + rotRightX), (xy_aut[1] + rotRightY), xy_manual[0], xy_manual[1]);

				if(distLeft > distRight) {
					autonomous_heading[0] = rotLeftX;
					autonomous_heading[1] = rotLeftY;
				} else {
					autonomous_heading[0] = rotRightX;
					autonomous_heading[1] = rotRightY;
				}
			} else {
				if (xy_aut[1] < xy_aut[0]) {
					autonomous_heading[0] = rotRightX;
					autonomous_heading[1] = rotRightY;
				} else {
					autonomous_heading[0] = rotLeftX;
					autonomous_heading[1] = rotLeftY;
				}
			}
			data[1] = (uint8_t)(15 - autonomous_heading[0]*8);
			data[2] = (uint8_t)(autonomous_heading[1]*8);

			printf("new heading: %f, %f\n", autonomous_heading[0], autonomous_heading[1]);

			// pthread_mutex_lock(&radio_lock);
			sendData(data, spi_dev, 3);
			// pthread_mutex_unlock(&radio_lock);
		}
	}
}

/*
 * 1----3
 * |	|
 * 2----4
 *
 *
 */


void getCoords(double *xy, int missingTower, double *last_tower_dist) {

	switch(missingTower + 1) {
		case 1:
			xy[0] = axisDistance(last_tower_dist[1], last_tower_dist[3]);
			xy[1] = axisDistance(last_tower_dist[3], last_tower_dist[2]);
			break;
		case 2:
			xy[0] = axisDistance(last_tower_dist[0], last_tower_dist[2]);
			xy[1] = axisDistance(last_tower_dist[3], last_tower_dist[2]);
			break;
		case 3:
			xy[0] = axisDistance(last_tower_dist[1], last_tower_dist[3]);
			xy[1] = axisDistance(last_tower_dist[2], last_tower_dist[0]);
			break;
		case 4:
			xy[0] = axisDistance(last_tower_dist[0], last_tower_dist[2]);
			xy[1] = axisDistance(last_tower_dist[1], last_tower_dist[0]);
			break;
	}
}

// rad2 is the from the "higher" receiver i.e. further on the axis
double axisDistance(double rad1, double rad2) {
	return -((rad2*rad2)-(rad1*rad1)-(TOWER_TO_TOWER*TOWER_TO_TOWER))/(2*TOWER_TO_TOWER);
}

void handleData(uint8_t *data) {
	// do somethign with data

	int time = (data[2] | (data[3] << 8));
	// double distance_cm = time * 0.01580 - 17.14;
	double theory = time * 0.01715;
	double distance_cm = theory * 0.9503 + 22.14;
	// printf("theory: %f\n", theory);
	// printf("got %f\n from id %d\n", distance_cm, data[1] - 0xb0);

	// printf("distance packet received\n");
	switch(data[1]) {
		case 0xb1:
			if((distance_cm < MAX_LENGTH && distance_cm > 0)) {
				last_tower_dist[0] = distance_cm;
				theory_dist[0] = theory;
				last_tower_ping[0] = 0;
			}
			break;
		case 0xb2:
			if((distance_cm < MAX_LENGTH && distance_cm > 0)) {
				last_tower_dist[1] = distance_cm;
				theory_dist[1] = theory;
				last_tower_ping[1] = 0;
			}
			break;
		case 0xb3:
			if((distance_cm < MAX_LENGTH && distance_cm > 0)) {
				last_tower_dist[2] = distance_cm;
				theory_dist[2] = theory;
				last_tower_ping[2] = 0;
			}
			break;
		case 0xb4:
			if((distance_cm < MAX_LENGTH && distance_cm > 0)) {
				last_tower_dist[3] = distance_cm;
				theory_dist[3] = theory;
				last_tower_ping[3] = 0;
			}
			break;
	}
	// printf("received tower %d\n", data[1]);
	// int time_us = (data[2] | (data[3] << 8));
	//double distance_cm = time_us*0.0343;
	// double regressed = distance_cm*1.780 - 46.16;
	// printf("non regressed: %f, regressed: %f\n", distance_cm, regressed);
}

void updateTimeSpec(struct timespec *ts, long nsPeriod) {
	if(ts->tv_nsec + nsPeriod > 1000000000) {
		ts->tv_sec++;
	}
	ts->tv_nsec = (ts->tv_nsec + nsPeriod)%1000000000;
}

void *sendPing(void *p) {
	spi *spi_dev = (spi*)p;

	// struct timespec tspec;
	// clock_gettime(CLOCK_REALTIME, &tspec);
	// long period = 1000000000;
	for(;;) {
		// printf("pinging...\n");
		usleep(200000);
		int manualTowers = pingCoords(xy_manual, spi_dev, SEND_PING_MANUAL, last_tower_dist);
		// int autonomousTowers = pingCoords(xy_aut, spi_dev, SEND_PING_AUTONOMOUS, last_tower_dist_aut);

		// changeDirection = (manualTowers > 0 || autonomousTowers > 0);
	}
	return (void *)0;
}

int pingCoords(double *xy, spi * spi_dev, uint8_t head, double *dist) {
	uint8_t myData[DATA_SIZE];
	myData[0] = head;
	myData[1] = 0xff;
	myData[2] = 0xff; // dummies
	uint8_t towerData[1];
	towerData[0] = SEND_PING_TOWER;
	// printf("pinging\n");
	// pthread_mutex_lock(&radio_lock);
	// printf("in mutex\n");
	// printf("ping result: %d\n", sendData(myData, spi_dev, 3));
	sendData(myData, spi_dev, 3);
	sendData(towerData, spi_dev, 1);
	// pthread_mutex_unlock(&radio_lock);
	// printf("ping sent\n");
	uint8_t gotResponse = 0;
	uint8_t sreg;
	int timeOut = 0;
	// reset whether they've been received
	for(uint8_t i = 0; i < 4; ++i) {
		last_tower_ping[i]++;
	}
	while(gotResponse < 3 && timeOut < 1000) {
		++timeOut;
		if(libsoc_gpio_get_level(irq) != HIGH) {
			libsoc_spi_rw(spi_dev, &GET_NRF_SREG, &sreg, 1);
			if(sreg & 0b01000000) {
				// printf("match!\n");
				uint8_t pipe[PIPE_SIZE + 1];
				// pthread_mutex_lock(&radio_lock);
				receiveData(spi_dev, pipe);
				resetIRQ(spi_dev);
				// pthread_mutex_unlock(&radio_lock);
				handleData(pipe);
				gotResponse++;
			}
		}

		usleep(10);
	}
	uint8_t recentPings = 0;
	uint8_t missingTower = 0;
	for(uint8_t i = 0; i < 4; ++i) {
		if(last_tower_ping[i] < 10) {
			++recentPings;
		} else {
			missingTower = i;
		}
	}
	if(recentPings >= 3) {
		// printf("missing tower %d\n", missingTower +1);
		getCoords(xy, missingTower, dist);
		printf("found robot at %f, %f\n", xy_manual[0], xy_manual[1]);
		// getCoords(xy, missingTower, theory_dist);
		// printf("theory found robot at %f, %f\n", xy_manual_theory[0], xy_manual_theory[1]);
	}
	return recentPings;
}


void *getPhone(void *p) {
	spi *spi_dev = (spi *) p;
	uint8_t data[DATA_SIZE];
	data[0] = MANUAL_MOVE;

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
			sendData(data, spi_dev, 3);
			// pthread_mutex_unlock(&radio_lock);
			printf("sent!\n");
		}
	}
	close(fd);
}

double getDist(double x1, double y1, double x2, double y2) {
	return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

double rotX(int right, double curx, double cury) {
	if(right) {
		return curx * 0.97 - cury * 0.174;
	} else {
		return curx * 0.97 + cury * 0.174;
	}
}

double rotY(int right, double curx, double cury) {

	if(right) {
		return curx*0.174 + cury*0.97;
	} else {
		return curx*-0.174 + cury*0.97;
	}
}
