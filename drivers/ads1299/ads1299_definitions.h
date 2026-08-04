/**
 * @file ads1299_definitions.h
 * @author Karl Gerber
 * @brief This file defines commands and reigsters of ads1299
 * @version 0.1
 * @date 2026-07-07
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once // Exclude multiple includes

#define CMD_WAKEUP  0x02
#define CMD_STANDBY 0x04
#define CMD_RESET   0x06
#define CMD_START   0x08
#define CMD_STOP    0x0A
#define CMD_RDATAC  0x10
#define CMD_SDATAC  0x11
#define CMD_RDATA   0x12

#define CMD_RREG    0x20
#define CMD_WREG    0x40

#define REG_ID      0x00
#define REG_CONFIG1 0x01
#define REG_CONFIG2 0x02
#define REG_CONFIG3 0x03
#define REG_LOFF    0x04
#define REG_CH1SET  0x05
#define REG_CH2SET  0x06
#define REG_CH3SET  0x07
#define REG_CH4SET  0x08
#define REG_CH5SET  0x09
#define REG_CH6SET  0x0A
#define REG_CH7SET  0x0B
#define REG_CH8SET  0x0C
#define REG_BIAS_SENSP  0x0D
#define REG_BIAS_SENSN  0x0E
#define REG_LOFF_SENSP  0x0F
#define REG_LOFF_SENSN  0x10
#define REG_LOFF_FLIP   0x11
#define REG_LOFF_STATP  0x12
#define REG_LOFF_STATN  0x13
#define REG_GPIO        0x14
#define REG_MISC1       0x15
#define REG_MISC2       0x16
#define REG_CONFIG4     0x17

#define ADS_NUM_DEVICES 2
#define ADS_NUM_CHANNELS    8 // daisy chain
#define ADS_BYTES_PER_DEVICE 27
#define ADS_DAISY_FRAME_BYTES (ADS_NUM_DEVICES * ADS_BYTES_PER_DEVICE)