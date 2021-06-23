#ifndef __RTU_MASTER_H__
#define __RTU_MASTER_H__
#include <stdio.h>
#include <stdint.h>
#include <string.h>

typedef enum
{
    FIELD_SLAVE_ID       = 0,
    FIELD_FUNCTION_CODE  = 1,
    FIELD_HI_REG_ADDR    = 2,
    FIELD_LO_REG_ADDR    = 3,
    FIELD_HI_REG_VAL     = 4,
    FIELD_LO_REG_VAL     = 5,
    FIELD_HI_CRC         = 6,
    FIELD_LO_CRC         = 7
} status_code;

static uint16_t crc_calc(uint8_t* data, int len);
void set_rtu_01_read_coils(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_num);
void set_rtu_03_read_holding_registers(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_num);
void set_rtu_05_write_single_coil(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_val);
void set_rtu_06_write_single_register(uint8_t *rtu_frame, uint8_t slave_id, uint16_t reg_addr, uint16_t reg_val);

#endif