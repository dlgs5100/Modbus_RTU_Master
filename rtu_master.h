/*
* \brief
* Modbus RTU Master API Implement Function 01, 03, 05, 06 Header
*
* \copyright
* Copyright (C) MOXA Inc. All rights reserved.
* This software is distributed under the terms of the
* MOXA License. See the file COPYING-MOXA for details.
*
* \date 2021/06/24
* First release
*
* \author 
* Jerry YH Cheng
*/

#ifndef __RTU_MASTER_H__
#define __RTU_MASTER_H__

/*****************************************************************************
* Include files
****************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*****************************************************************************
* Private types/enumerations/variables/define
****************************************************************************/

/* Enum of read and write single register function frame field */
typedef enum
{
    R_W_SINGLE_FIELD_SLAVE_ID       = 0,
    R_W_SINGLE_FIELD_FUNCTION_CODE  = 1,
    R_W_SINGLE_FIELD_HI_REG_ADDR    = 2,
    R_W_SINGLE_FIELD_LO_REG_ADDR    = 3,
    R_W_SINGLE_FIELD_HI_REG_VAL     = 4,
    R_W_SINGLE_FIELD_LO_REG_VAL     = 5,
    R_W_SINGLE_FIELD_HI_CRC         = 6,
    R_W_SINGLE_FIELD_LO_CRC         = 7
}r_w_single;

#define R_W_SINGLE_CRC_LEN 6    /**< Length of read and write single register function calculate by crc in frame */

/*****************************************************************************
* Private function declaration
****************************************************************************/

static uint16_t crc_calc(uint8_t* data, int len);

/*****************************************************************************
* Public function declaration
****************************************************************************/

void set_rtu_01_read_coils(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_num);
void set_rtu_03_read_holding_registers(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_num);
void set_rtu_05_write_single_coil(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_val);
void set_rtu_06_write_single_register(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_val);

#endif