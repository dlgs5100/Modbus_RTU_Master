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
#define BAUD                                 19200  /**< User define baud rate */

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
    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);
    /* Setting control option */
    options.c_cflag |= CREAD; /* Open receive */
    options.c_cflag |= CLOCAL; /* Ignore control line(Avoid occupy port) */
    options.c_cflag &= ~PARENB; /* Disable parity */
    options.c_cflag &= ~CSTOPB; /* Disable two stop bits(Only one)*/
    options.c_cflag &= ~CSIZE; /* Clear bit size*/
    options.c_cflag |= CS8; /* Set  8 bits per byte */
    options.c_cflag |= CRTSCTS; /* Hardware flow control */
    /* Setting local option */
    options.c_lflag &= ~ICANON; /* Disable canonical mode(Using Raw input) */
    options.c_lflag &= ~ECHO; /* Disable echo */
    options.c_lflag &= ~ECHOE; /* Disable erase */
    options.c_lflag &= ~ISIG; /* Disable interpretation of INTR, QUIT, SUSP and DSUSP */
    /* Setting output option */
    options.c_oflag &= ~OPOST; /* Raw output */
    /* Setting control character */
    options.c_cc[VMIN] = 0; /* Min waiting bytes to response() */
    options.c_cc[VTIME] = 10; /* Timeout to response read() */

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
    int serial_fd, request_size, response_size, total_receive_size, t1_5, t3_5;
    int function_code = 6, slave_id = 1, reg_address = 0, reg_quantity = 1, output_val = 456;       // Test variable
    struct timeval timeout, sys_time;
    fd_set read_fds_master, read_fds;
    uint8_t *request = NULL, *response = NULL;
    long int begin_request_timer = 0;

    if ((serial_fd = set_serial()) == ERROR_RETURN)
    {
        goto close_free;
    }

    t1_5 = BAUD > 19200 ? 750 : (15000000 / BAUD);
    t3_5 = BAUD > 19200 ? 1750 : (35000000 / BAUD);

    // FD_ZERO(&read_fds_master);
    // FD_ZERO(&read_fds);

    // FD_SET(serial_fd, &read_fds_master);

    /* Initialize timeout timer to send request*/
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    while (1)
    {
        // read_fds = read_fds_master;
        FD_ZERO(&read_fds); //每次循环都要清空集合，否则不能检测描述符变化
        FD_SET(serial_fd,&read_fds); //添加描述符

        /* Select active fds into read_fds */
        switch (select(serial_fd + 1, &read_fds, NULL, NULL, &timeout))
        {
        case -1:
            perror("select()");
            goto close_free;

        case 0: /* Timeout happened(Include first time setting timeout) */
            
            /* Inter-frame delay */
            usleep(t3_5);
            if(response)
            {   
                if(response_size == total_receive_size)   /* Handle incomplete response frame */
                {
                    DEBUG("Processing complete frame");
                }
                else
                {
                    DEBUG("Delete incomplete Frame");
                }
                
                free(response);
                response = NULL;
            }

            gettimeofday(&sys_time, NULL);

            if (!begin_request_timer || (sys_time.tv_sec * 1000 + sys_time.tv_usec / 1000) - begin_request_timer >= POLL_INTERVAL) /* Handle first request and polling interval */
            {
                /* Set Modbus RTU frame */
                switch (function_code)
                {
                case MODBUS_FUNCTION_CODE_01:
                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = F01_F03_WITHOUT_INPUT_RESPONSE_SIZE + (int)ceilf((float)reg_quantity / 8.0); /* Setting size by Read Quantity(bits to byte)*/
                    request = calloc(sizeof(uint8_t), request_size);

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
                    request = calloc(sizeof(uint8_t), request_size);

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
                    request = calloc(sizeof(uint8_t), request_size);

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
                    request = calloc(sizeof(uint8_t), request_size);

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
                DEBUG("request:");
                for(int i=0; i<request_size; i++)
                {
                    DEBUG("%02x", request[i]);
                }


                tcflush(serial_fd, TCOFLUSH);
                ioctl(serial_fd, TCFLSH, 1);

                /* Update polling being timer */
                gettimeofday(&sys_time, NULL);
                begin_request_timer = sys_time.tv_sec * 1000 + sys_time.tv_usec / 1000;

                /* Update timeout timer */
                timeout.tv_sec = RESPONSE_TIMEOUT / 1000;
                timeout.tv_usec = (RESPONSE_TIMEOUT % 1000) * 1000;

                /* Inter-Frame delay */
                usleep(t3_5);

                /* Initialize receive_size */
                total_receive_size = 0;

                free(request);
            }

            break;

        default:
            if (FD_ISSET(serial_fd, &read_fds))
            {
                int receive_size;

                response = calloc(sizeof(uint8_t), response_size);

                if (!response)
                {
                    perror("calloc()");
                    goto close_free;
                }
                
                if ((receive_size = read(serial_fd, response + total_receive_size, response_size)) == ERROR_RETURN)
                {
                    perror("read()");
                    goto close_free;
                }
                tcflush(serial_fd, TCIFLUSH);
                ioctl(serial_fd, TCFLSH, 0);
                
                DEBUG("receive_size: %d", receive_size);
                for(int i=0; i<receive_size; i++)
                {
                    DEBUG("%02x", response[i]);
                }

                total_receive_size += receive_size;

                if (total_receive_size < response_size)   /* Set timeout timer(t1.5) to receive remaining response */
                {
                    timeout.tv_sec = 0;
                    timeout.tv_usec = t1_5;
                }
                else    /* Clear timeout timer to send new request*/
                {
                    timeout.tv_sec = 0;
                    timeout.tv_usec = 0;
                }
            }

            break;
        }
    }
    // int receive_size;
    // request_size = R_W_SINGLE_REQUEST_SIZE;
    // response_size = W_RESPONSE_SIZE;
    // request = calloc(sizeof(uint8_t), request_size);
    // response = calloc(sizeof(uint8_t), response_size);

    // if (!request)
    // {
    //     perror("calloc()");
    //     goto close_free;
    // }
    // if (!response)
    // {
    //     perror("calloc()");
    //     goto close_free;
    // }

    // set_rtu_06_write_single_register(request, slave_id, reg_address, output_val);
    // // for(int i=0; i<request_size; i++)
    // // {
    // //     request[i] = 0xa3;
    // // }
    // for(int i=0; i<request_size; i++)
    // {
    //     DEBUG("%02x", request[i]);
    // }
    // if (write(serial_fd, request, request_size) == ERROR_RETURN)
    // {
    //     perror("write()");
    //     goto close_free;
    // }
    // if ((receive_size = read(serial_fd, response, response_size)) == ERROR_RETURN)
    // {
    //     perror("read()");
    //     goto close_free;
    // }
    
    // DEBUG("receive_size: %d", receive_size);
    // for(int i=0; i<receive_size; i++)
    // {
    //     DEBUG("%02x", response[i]);
    // }

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