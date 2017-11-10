/*
 * BT.h
 *
 *  Created on: Nov 10, 2017
 *      Author: esdev
 */

#ifndef BT_H_
#define BT_H_

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


#define UART_OPEN_ERROR 0

int uartInit(void);
char uartRead(int fd);


#endif /* BT_H_ */
