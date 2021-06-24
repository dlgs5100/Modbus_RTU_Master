/*
* \brief
* Modbus RTU Master Implement Function 01, 03, 05, 06
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

/*****************************************************************************
 * Private functions
 ****************************************************************************/

static uint16_t crc_calc(uint8_t* data, int len)
{
    int j;
    unsigned int reg_crc=0xFFFF;
    while(len--)
    {
        reg_crc ^= *data++;
        for(j=0;j<8;j++)
        {
            if(reg_crc & 0x01) /* LSB(b0)=1 */
                reg_crc=(reg_crc>>1) ^ 0xA001;
            else
                reg_crc=reg_crc >>1;
        }
    }
    return reg_crc;
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/

void set_rtu_01_read_coils(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_num)
{
    rtu_frame[R_W_SINGLE_FIELD_SLAVE_ID] = slave_id;
    rtu_frame[R_W_SINGLE_FIELD_FUNCTION_CODE] = 0x01;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_ADDR] = (reg_addr>>8)&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_ADDR] = reg_addr&0xff;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_VAL] = (reg_num>>8)&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_VAL] = reg_num&0xff;
    
    uint16_t crc_tmp = crc_calc(rtu_frame, R_W_SINGLE_CRC_LEN);
    rtu_frame[R_W_SINGLE_FIELD_HI_CRC] = crc_tmp&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_CRC] = (crc_tmp>>8)&0xff;
}

void set_rtu_03_read_holding_registers(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_num)
{
    rtu_frame[R_W_SINGLE_FIELD_SLAVE_ID] = slave_id;
    rtu_frame[R_W_SINGLE_FIELD_FUNCTION_CODE] = 0x03;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_ADDR] = (reg_addr>>8)&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_ADDR] = reg_addr&0xff;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_VAL] = (reg_num>>8)&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_VAL] = reg_num&0xff;
    
    uint16_t crc_tmp = crc_calc(rtu_frame, R_W_SINGLE_CRC_LEN);
    rtu_frame[R_W_SINGLE_FIELD_HI_CRC] = crc_tmp&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_CRC] = (crc_tmp>>8)&0xff;
}

void set_rtu_05_write_single_coil(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_val)
{
    rtu_frame[R_W_SINGLE_FIELD_SLAVE_ID] = slave_id;
    rtu_frame[R_W_SINGLE_FIELD_FUNCTION_CODE] = 0x05;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_ADDR] = (reg_addr>>8)&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_ADDR] = reg_addr&0xff;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_VAL] = reg_val ? 0xff : 0x00;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_VAL] = 0x00;
    
    uint16_t crc_tmp = crc_calc(rtu_frame, R_W_SINGLE_CRC_LEN);
    rtu_frame[R_W_SINGLE_FIELD_HI_CRC] = crc_tmp&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_CRC] = (crc_tmp>>8)&0xff;
}

void set_rtu_06_write_single_register(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_val)
{
    rtu_frame[R_W_SINGLE_FIELD_SLAVE_ID] = slave_id;
    rtu_frame[R_W_SINGLE_FIELD_FUNCTION_CODE] = 0x06;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_ADDR] = (reg_addr>>8)&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_ADDR] = reg_addr&0xff;
    rtu_frame[R_W_SINGLE_FIELD_HI_REG_VAL] = (reg_val>>8)&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_REG_VAL] = reg_val&0xff;
    
    uint16_t crc_tmp = crc_calc(rtu_frame, R_W_SINGLE_CRC_LEN);
    rtu_frame[R_W_SINGLE_FIELD_HI_CRC] = crc_tmp&0xff;
    rtu_frame[R_W_SINGLE_FIELD_LO_CRC] = (crc_tmp>>8)&0xff;
}