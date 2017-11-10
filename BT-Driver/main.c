#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <time.h>
//#include <iostream>  // this is not C++ so this line not needed
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <limits.h>

int main() {


	int fd, rx_length;
	char charRead;

	printf("starting\n");

	//open uart1 for rx
	if((fd = open("/dev/ttyS1", O_RDONLY | O_NOCTTY|O_NONBLOCK)) < 0)
	{
		perror("open failed for /dev/ttyS1");
		exit(1);
	}

	struct termios uart1;
	cfsetospeed(&uart1, B9600);

	uart1.c_iflag = 0;
	uart1.c_oflag = 0;
	uart1.c_lflag = 0;
	tcsetattr(fd, TCSANOW, &uart1);

	while(1) {
		rx_length = read(fd, &charRead, 1);

		if(rx_length > 0) {
			printf("received\n");
			printf("%c\n", charRead);
		}
	}

	return 0;
}
