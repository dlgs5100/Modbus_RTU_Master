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
#define DEBUG(format,...) printf(format, ##__VA_ARGS__)
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

typedef enum
{
    OK_STATUS                      = 0,
    SLAVE_ID_OUT_OF_RANGE_STATUS,
    REG_ADDR_OUT_OF_RANGE_STATUS,
    REG_NUM_OUT_OF_RANGE_STATUS,
    REG_VAL_OUT_OF_RANGE_STATUS,
    STATUS_AMOUNT_CAP
} return_status;

#define ERROR_RETURN                            -1  /**< Return error */
#define OK_STATUS                                0  /**< Return error */
#define R_W_SINGLE_REQUEST_SIZE                  8  /**< Length of read and write single register function request */
#define W_RESPONSE_SIZE                          8  /**< Length of write register function response */
#define F01_F03_WITHOUT_INPUT_RESPONSE_SIZE      5  /**< Length of 01,03 function response */
#define F01_F03_RESPONSE_BYTE_COUNT_FIELD        2  /**< Field byte count of 01,03 function response */
#define POLL_INTERVAL                         1000  /**< User define polling interval */
#define RESPONSE_TIMEOUT                       500  /**< User define response timeout */
#define BAUD                                 19200  /**< User define baud rate */

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static int set_serial()
{

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

static int collision_detect(uint8_t *request, int request_size, uint8_t *response, int response_size)
{
    int index_field;
    uint16_t request_quantity;

    if (request[R_W_SINGLE_FIELD_FUNCTION_CODE] == MODBUS_FUNCTION_CODE_01)
    {   
        request_quantity = ((uint16_t)request[R_W_SINGLE_FIELD_HI_REG_VAL] << 8) | request[R_W_SINGLE_FIELD_LO_REG_VAL];
        
        if (request[R_W_SINGLE_FIELD_SLAVE_ID] != response[R_W_SINGLE_FIELD_SLAVE_ID] || 
        request[R_W_SINGLE_FIELD_FUNCTION_CODE] != response[R_W_SINGLE_FIELD_FUNCTION_CODE] ||
        (int)ceilf((float)request_quantity / 8.0) != response[F01_F03_RESPONSE_BYTE_COUNT_FIELD])
        {
            return 1;
        }
    }
    else if (request[R_W_SINGLE_FIELD_FUNCTION_CODE] == MODBUS_FUNCTION_CODE_03)
    {
        request_quantity = ((uint16_t)request[R_W_SINGLE_FIELD_HI_REG_VAL] << 8) | request[R_W_SINGLE_FIELD_LO_REG_VAL];

        if (request[R_W_SINGLE_FIELD_SLAVE_ID] != response[R_W_SINGLE_FIELD_SLAVE_ID] || 
        request[R_W_SINGLE_FIELD_FUNCTION_CODE] != response[R_W_SINGLE_FIELD_FUNCTION_CODE] ||
        request_quantity * 2 != response[F01_F03_RESPONSE_BYTE_COUNT_FIELD])
        {
            return 1;
        }
    }
    else if (request[R_W_SINGLE_FIELD_FUNCTION_CODE] == MODBUS_FUNCTION_CODE_05 || request[R_W_SINGLE_FIELD_FUNCTION_CODE] == MODBUS_FUNCTION_CODE_06)
    {
        for (index_field=0; index_field<request_size; index_field++)
        {
            if (request[index_field] != response[index_field])
            {
                return 1;
            }
        }
    }

    return 0;
}

static const char* err2msg(int code)
{
    static char default_error[] = "Unknown Error";
    static char* error_message[] = {
    "",
    "Input slave id between 0 and 255",
    "Input start address between 0 and 65535",
    "Out of range address",
    "Out of range number"
    };

    if (code >= STATUS_AMOUNT_CAP)
    {
        return default_error;
    }
    return error_message[code];
}

/*****************************************************************************
* Public function declaration
****************************************************************************/

int main()
{
    int serial_fd, request_size, response_size, total_receive_size, t1_5, t3_5, status_code;
    int function_code = 5, slave_id = 2, reg_address = 0, reg_quantity = 1, output_val = 1;       // Test variable
    int remaining_timeout_sec, remaining_timeout_usec;
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


    /* Initialize timeout timer to send request*/
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    while (1)
    {
        FD_ZERO(&read_fds);
        FD_SET(serial_fd,&read_fds);
        
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
                if(response[R_W_SINGLE_FIELD_SLAVE_ID] != slave_id) /* Handle wrong slave id response */
                {
                    /* Timeout timer recovery */
                    timeout.tv_sec = remaining_timeout_sec;
                    timeout.tv_usec = remaining_timeout_usec;
                    break;
                }

                if(response_size == total_receive_size)   /* Handle incomplete response frame */
                {
                    if (collision_detect(request, request_size, response, response_size))
                    {
                        DEBUG("Collision detect\n");
                    }
                    else
                    {
                        DEBUG("Processing complete frame\n");
                    }
                }
                else
                {
                    DEBUG("Delete incomplete Frame\n");
                }
                
                free(request);
                free(response);
                request = NULL;
                response = NULL;
            }

            gettimeofday(&sys_time, NULL);

            if (!begin_request_timer || (sys_time.tv_sec * 1000 + sys_time.tv_usec / 1000) - begin_request_timer >= POLL_INTERVAL) /* Handle first request and polling interval */
            {
                /* Set Modbus RTU frame */
                switch (function_code)
                {
                case MODBUS_FUNCTION_CODE_01:

                    if (slave_id < 0 || slave_id > 255)
                    {
                        status_code = SLAVE_ID_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (reg_address < 0 || reg_address > 65535)
                    {
                        status_code = REG_ADDR_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (reg_quantity < 0 || reg_quantity > 2000)
                    {
                        status_code = REG_NUM_OUT_OF_RANGE_STATUS;
                        break;
                    }

                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = F01_F03_WITHOUT_INPUT_RESPONSE_SIZE + (int)ceilf((float)reg_quantity / 8.0); /* Setting size by Read Quantity(bits to byte)*/
                    request = calloc(sizeof(uint8_t), request_size);

                    if (!request)
                    {
                        perror("calloc()");
                        goto close_free;
                    }

                    set_rtu_01_read_coils(request, slave_id, reg_address, reg_quantity);
                    status_code = OK_STATUS;

                    break;

                case MODBUS_FUNCTION_CODE_03:

                    if (slave_id < 0 || slave_id > 255)
                    {
                        status_code = SLAVE_ID_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (reg_address < 0 || reg_address > 65535)
                    {
                        status_code = REG_ADDR_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (reg_quantity < 0 || reg_quantity > 125)
                    {
                        status_code = REG_NUM_OUT_OF_RANGE_STATUS;
                        break;
                    }

                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = F01_F03_WITHOUT_INPUT_RESPONSE_SIZE + reg_quantity * 2; /* Setting size by Read Quantity(two bytes) */
                    request = calloc(sizeof(uint8_t), request_size);

                    if (!request)
                    {
                        perror("calloc()");
                        goto close_free;
                    }

                    set_rtu_03_read_holding_registers(request, slave_id, reg_address, reg_quantity);
                    status_code = OK_STATUS;

                    break;

                case MODBUS_FUNCTION_CODE_05:

                    if (slave_id < 0 || slave_id > 255)
                    {
                        status_code = SLAVE_ID_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (reg_address < 0 || reg_address > 65535)
                    {
                        status_code = REG_ADDR_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (output_val != 0 && output_val != 1)
                    {
                        status_code = REG_VAL_OUT_OF_RANGE_STATUS;
                        break;
                    }

                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = W_RESPONSE_SIZE;
                    request = calloc(sizeof(uint8_t), request_size);

                    if (!request)
                    {
                        perror("calloc()");
                        goto close_free;
                    }

                    set_rtu_05_write_single_coil(request, slave_id, reg_address, output_val);
                    status_code = OK_STATUS;

                    break;

                case MODBUS_FUNCTION_CODE_06:

                    if (slave_id < 0 || slave_id > 255)
                    {
                        status_code = SLAVE_ID_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (reg_address < 0 || reg_address > 65535)
                    {
                        status_code = REG_ADDR_OUT_OF_RANGE_STATUS;
                        break;
                    }
                    else if (output_val < 0 || output_val > 65535)
                    {
                        status_code = REG_VAL_OUT_OF_RANGE_STATUS;
                        break;
                    }

                    request_size = R_W_SINGLE_REQUEST_SIZE;
                    response_size = W_RESPONSE_SIZE;
                    request = calloc(sizeof(uint8_t), request_size);

                    if (!request)
                    {
                        perror("calloc()");
                        goto close_free;
                    }

                    set_rtu_06_write_single_register(request, slave_id, reg_address, output_val);
                    status_code = OK_STATUS;

                    break;

                default:
                    fprintf(stderr, "Error Function Code\n");
                    goto close_free;
                }
                
                if (status_code != OK_STATUS)
                {
                    DEBUG("%s\n", err2msg(status_code));
                    goto close_free;
                }

                if (write(serial_fd, request, request_size) == ERROR_RETURN)
                {
                    perror("write()");
                    goto close_free;
                }
                DEBUG("=================\n");
                DEBUG("request:\n");
                for(int i=0; i<request_size; i++)
                {
                    DEBUG("%02x ", request[i]);
                }
                DEBUG("\n");

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
                
                DEBUG("receive_size: %d\n", receive_size);
                for(int i=0; i<receive_size; i++)
                {
                    DEBUG("%02x ", response[i]);
                }
                DEBUG("\n");

                total_receive_size += receive_size;

                if (total_receive_size < response_size)   /* Set timeout timer(t1.5) to receive remaining response */
                {
                    timeout.tv_sec = 0;
                    timeout.tv_usec = t1_5;
                }
                else    /* Clear timeout timer to send new request*/
                {
                    /* Save remaining timeout timer for recovery from receive wrong slave id response */
                    remaining_timeout_sec = timeout.tv_sec;
                    remaining_timeout_usec = timeout.tv_usec;

                    timeout.tv_sec = 0;
                    timeout.tv_usec = 0;
                }
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