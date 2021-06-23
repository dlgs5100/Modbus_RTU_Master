#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>

#include "rtu_master.h"

#define ERROR_RETURN -1

static int set_serial(){

    int serial_fd;
    struct termios options;

    if((serial_fd = open("/dev/ttyUSB0", O_RDWR)) < 0)
    {
        perror("open()");
        return ERROR_RETURN;
    }
    else
    {
        fcntl(serial_fd, F_SETFL, 0);
    }

    struct serial_struct serial;
    ioctl(serial_fd, TIOCGSERIAL, &serial); 
    serial.flags |= ASYNC_LOW_LATENCY; // (0x2000)
    ioctl(serial_fd, TIOCSSERIAL, &serial);

    /* Setting I/O Baud Rate */
    cfsetispeed(&options, B9600);
    cfsetospeed(&options, B9600);
    /* Setting control fleid */
    options.c_cflag |= CREAD; /* Open receive */
    options.c_cflag |= CLOCAL; /* Ignore control line(Avoid occupy port) */
    options.c_cflag &= ~PARENB; /* Disable parity */
    options.c_cflag &= ~CSTOPB; /* Disable two stop bits(only one)*/
    options.c_cflag &= ~CSIZE; /* Clear bit size*/
    options.c_cflag |= CS8; /* Set  8 bits per byte */
    options.c_cflag |= CRTSCTS; /* Hardware flow control */
    /* Setting local field */
    options.c_lflag &= ~ICANON; /* Disable canonical mode*/
    options.c_lflag &= ~ECHO; /* Disable echo */
    options.c_lflag &= ~ECHOE; /* Disable erase */
    options.c_lflag &= ~ECHONL; /* Disable new line echo */
    options.c_lflag &= ~ISIG; /* Disable interpretation of INTR, QUIT, SUSP and DSUSP */

    options.c_oflag &= ~OPOST;
    options.c_oflag &= ~ONLCR;

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 10;

    if (tcsetattr(serial_fd, TCSANOW, &options) < 0)
    {
        perror("tcsetattr()");
        return ERROR_RETURN;
    }

    return serial_fd;
}
int main()
{
    int serial_fd;
    uint8_t send_data[8] = { 0 };
    uint8_t rcv_data[32] = { 0 };

    if((serial_fd = set_serial()) == ERROR_RETURN)
    {
        printf("Error test\n");
    }

    memset(send_data, 0, sizeof(send_data));
	set_rtu_03_read_holding_registers(send_data, 1, 0, 1);
    write(serial_fd, send_data, 8);	
    read(serial_fd, rcv_data, sizeof(rcv_data));
    for (int i = 0; i < sizeof(rcv_data); i++)
	{
		printf("%02x ", rcv_data[i]);
	}
    printf("\n");

    return 0;
}