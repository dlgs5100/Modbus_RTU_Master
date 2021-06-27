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

#define __DEBUG__

#ifdef __DEBUG__
#define DEBUG(format,...) printf(format"\n", ##__VA_ARGS__)
#else
#define DEBUG(format,...)
#endif

/*****************************************************************************
* Include files
****************************************************************************/

#include "rtu_master.h"
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/serial.h>

/*****************************************************************************
* Private types/enumerations/variables/define
****************************************************************************/

typedef enum
{
    MODBUS_FUNCTION_CODE_01         = 1,
    MODBUS_FUNCTION_CODE_03         = 3,
    MODBUS_FUNCTION_CODE_05         = 5,
    MODBUS_FUNCTION_CODE_06         = 6
} function_code;

#define ERROR_RETURN                            -1  /**< Return error */
#define R_W_SINGLE_REQUEST_SIZE                  8  /**< Length of read and write single register function request */
#define W_RESPONSE_SIZE                          8  /**< Length of write register function response */
#define F01_F03_WITHOUT_INPUT_RESPONSE_SIZE      5  /**< Length of 01,03 function response */
#define POLL_INTERVAL                         1000  /**< User define polling interval */
#define RESPONSE_TIMEOUT                       500  /**< User define response timeout */

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static int set_serial()
{

    int serial_fd;
    struct termios options;

    if ((serial_fd = open("/dev/ttyUSB0", O_RDWR)) < 0)
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
    int function_code = 6, slave_id = 1, reg_address = 0, reg_quantity = 1, output_val = 465;       // Test variable
    struct timeval timeout, sys_time;
    fd_set read_fds_master, read_fds;
    uint8_t *request = NULL, *response = NULL;
    long int begin_request_timer = 0;

    if ((serial_fd = set_serial()) == ERROR_RETURN)
    {
        goto close_free;
    }

    FD_ZERO(&read_fds_master);
    FD_ZERO(&read_fds);

    FD_SET(serial_fd, &read_fds_master);

    /* Initialize timeout timer to send request*/
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    while (1)
    {
        read_fds = read_fds_master;

        /* Select active fds into read_fds */
        switch (select(serial_fd + 1, &read_fds, NULL, NULL, &timeout))
        {
        case -1:
            perror("select()");
            goto close_free;

        case 0: /* Timeout happened(Include first time setting timeout) */
            gettimeofday(&sys_time, NULL);

            if (!begin_request_timer || (sys_time.tv_sec * 1000 + sys_time.tv_usec / 1000) - begin_request_timer >= POLL_INTERVAL) /* Handle first request and polling interval */
            {
                /* Set Modbus RTU frame */
                switch (function_code)
                {
                case MODBUS_FUNCTION_CODE_01:
                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = F01_F03_WITHOUT_INPUT_RESPONSE_SIZE + (int)ceilf((float)reg_quantity / 8.0); /* Setting size by Read Quantity(bits to byte)*/
                    request = malloc(sizeof(uint8_t) * request_size);

                    if (!request)
                    {
                        perror("malloc()");
                        goto close_free;
                    }

                    set_rtu_01_read_coils(request, slave_id, reg_address, reg_quantity);
                    break;

                case MODBUS_FUNCTION_CODE_03:
                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = F01_F03_WITHOUT_INPUT_RESPONSE_SIZE + reg_quantity * 2; /* Setting size by Read Quantity(two bytes) */
                    request = malloc(sizeof(uint8_t) * request_size);

                    if (!request)
                    {
                        perror("malloc()");
                        goto close_free;
                    }

                    set_rtu_03_read_holding_registers(request, slave_id, reg_address, reg_quantity);
                    break;

                case MODBUS_FUNCTION_CODE_05:
                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = W_RESPONSE_SIZE;
                    request = malloc(sizeof(uint8_t) * request_size);

                    if (!request)
                    {
                        perror("malloc()");
                        goto close_free;
                    }

                    set_rtu_05_write_single_coil(request, slave_id, reg_address, output_val);
                    break;

                case MODBUS_FUNCTION_CODE_06:
                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = W_RESPONSE_SIZE;
                    request = malloc(sizeof(uint8_t) * request_size);

                    if (!request)
                    {
                        perror("malloc()");
                        goto close_free;
                    }

                    set_rtu_06_write_single_register(request, slave_id, reg_address, output_val);
                    break;

                default:
                    fprintf(stderr, "Error Function Code\n");
                    goto close_free;
                }

                if (write(serial_fd, request, request_size) == ERROR_RETURN)
                {
                    perror("write()");
                    goto close_free;
                }

                /* Update polling being timer */
                gettimeofday(&sys_time, NULL);
                begin_request_timer = sys_time.tv_sec * 1000 + sys_time.tv_usec / 1000;

                /* Update timeout timer */
                timeout.tv_sec = RESPONSE_TIMEOUT / 1000;
                timeout.tv_usec = (RESPONSE_TIMEOUT % 1000) * 1000;

                free(request);
            }

            break;

        default:
            if (FD_ISSET(serial_fd, &read_fds))
            {
                response = calloc(sizeof(uint8_t), response_size);

                if (!response)
                {
                    perror("calloc()");
                    goto close_free;
                }

                if (read(serial_fd, response, response_size) == ERROR_RETURN)
                {
                    perror("read()");
                    goto close_free;
                }

                /* Clear timeout timer to send new request*/
                timeout.tv_sec = 0;
                timeout.tv_usec = 0;

                free(response);
            }

            break;
        }
    }

close_free:
    close(serial_fd);

    if (request)
    {
        free(request);
    }

    if (response)
    {
        free(response);
    }

    return 0;
}