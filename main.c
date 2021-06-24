/*
* \brief
* Modbus RTU Master API Test
*
* \copyright
* Copyright (C) MOXA Inc. All rights reserved.
* This software is distributed under the terms of the
* MOXA License. See the file COPYING-MOXA for details.
* \date 2021/06/24
* First release
* \author Jerry YH Cheng
*/

/*****************************************************************************
* Include files
****************************************************************************/

#include "rtu_master.h"
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h> 
#include <linux/serial.h>

/*****************************************************************************
* Private types/enumerations/variables/define
****************************************************************************/

#define ERROR_RETURN -1
#define R_W_SINGLE_REQUEST_SIZE 8
#define W_RESPONSE_SIZE 8
#define F01_F03_WITHOUT_INPUT_RESPONSE_SIZE 5

/*****************************************************************************
 * Private functions
 ****************************************************************************/

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

/*****************************************************************************
* Public function declaration
****************************************************************************/

int main()
{
    int serial_fd, request_size, response_size;
    int function_code = 6, slave_id = 1, reg_address = 0, reg_quantity = 1, output_val = 465, poll_interval, response_timeout;
    uint8_t *request, *response;

    if((serial_fd = set_serial()) == ERROR_RETURN)
    {
        goto fd_close;
    }

    memset(request, 0, sizeof(request));
    switch (function_code)
    {
    case 1:
        request_size = R_W_SINGLE_REQUEST_SIZE;
        response_size = F01_F03_WITHOUT_INPUT_RESPONSE_SIZE+(int)ceilf((float)reg_quantity/8.0);
        request = malloc(sizeof(uint8_t) * request_size);

        set_rtu_01_read_coils(request, slave_id, reg_address, reg_quantity);

        break;
    case 3:
        request_size = R_W_SINGLE_REQUEST_SIZE;
        response_size = F01_F03_WITHOUT_INPUT_RESPONSE_SIZE+reg_quantity*2;
        request = malloc(sizeof(uint8_t) * request_size);

        set_rtu_03_read_holding_registers(request, slave_id, reg_address, reg_quantity);
        break;
    case 5:
        request_size = R_W_SINGLE_REQUEST_SIZE;
        response_size = W_RESPONSE_SIZE;
        request = malloc(sizeof(uint8_t) * request_size);

        set_rtu_05_write_single_coil(request, slave_id, reg_address, output_val);
        break;
    case 6:
        request_size = R_W_SINGLE_REQUEST_SIZE;
        response_size = W_RESPONSE_SIZE;
        request = malloc(sizeof(uint8_t) * request_size);

        set_rtu_06_write_single_register(request, slave_id, reg_address, output_val);
        break;
    }
    if(write(serial_fd, request, request_size) == ERROR_RETURN)
    {
        perror("write()");
        goto fd_close;
    }
    printf("Request: \n");
    for (int i = 0; i < request_size; i++)
	{
		printf("%02x ", request[i]);
	}
    printf("\n");

    response = calloc(sizeof(uint8_t), response_size);
    if(read(serial_fd, response, response_size) == ERROR_RETURN)
    {
        perror("read()");
        goto fd_close;
    }
    printf("Response: \n");
    for (int i = 0; i < response_size; i++)
	{
		printf("%02x ", response[i]);
	}
    printf("\n");

fd_close:
    close(serial_fd);
    if(request)
    {
        free(request);
    }
    if(response)
    {
        free(response);
    }

    return 0;
}