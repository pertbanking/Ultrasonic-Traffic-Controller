/* 
 * File:   NRF_consts.h
 * Author: Liam
 *
 * Created on May 16, 2017, 5:43 PM
 */

#ifndef NRF_CONSTS_H
#define	NRF_CONSTS_H

#include <stdint.h>

#ifdef	__cplusplus
extern "C" {
#endif

#define REGISTER_MASK 0b00011111 // AND this w/ addresses for config registers
#define W_MASK 0b00100000 // OR this w/ masked addresses
#define R_MASK 0x00 // OR this w/ masked addresses

// spi command for reading/writing payloads
#define R_RX_PAYLOAD 0b01100001
#define W_TX_PAYLOAD 0b10100000

#define GET_SREG 0xFF

// addresses for registers that we care about
#define CONFIG 0x00
#define EN_AA 0x01
#define EN_RX_ADDR 0x02
#define SETUP_AW 0x03
#define STATUS 0x07
const uint8_t RX_ADDR_P[6] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
const uint8_t TX_ADDR = 0x10;
const uint8_t RX_PW_P[6] = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16};
const uint8_t MY_ADDRESS[4] = {0xcf, 0x83, 0xf2, 0xf5};


#ifdef	__cplusplus
}
#endif

#endif	/* NRF_CONSTS_H */

