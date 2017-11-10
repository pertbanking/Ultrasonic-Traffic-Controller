/*
 * BT.c
 *
 *  Created on: Nov 10, 2017
 *      Author: esdev
 */
#include "BT.h"


int uartInit(void) {
	int fd;
	if((fd = open("/dev/ttyS1", O_RDONLY | O_NOCTTY|O_NONBLOCK)) < 0)
		{
			perror("open failed for /dev/ttyS1");
			return UART_OPEN_ERROR;
		}
	struct termios uart1;
	cfsetospeed(&uart1, B9600);

	uart1.c_iflag = 0;
	uart1.c_oflag = 0;
	uart1.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &uart1);

	return fd;
}

char uartRead(int fd) {
	char charRead;
	int rx_length;
	rx_length = read(fd, &charRead, 1);
	return uartRead;
}
