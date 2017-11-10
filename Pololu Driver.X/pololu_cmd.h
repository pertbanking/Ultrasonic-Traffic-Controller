/* 
 * File:   pololu_cmd.h
 * Author: Liam
 *
 * Created on October 30, 2017, 12:41 PM
 */

#ifndef POLOLU_CMD_H
#define	POLOLU_CMD_H

#include <stdint.h>
#include <xc.h>

#ifdef	__cplusplus
extern "C" {
#endif
    
#define _XTAL_FREQ 16000000UL
#define PACKET_SIZE 4
#define TX LATCbits.LATC5
#define BIT_BANG_DIR TRISCbits.TRISC5
#define MAX_SPEED 10
#define PACKET_START 12
#define PACKET_END 11
#define Baudrate 9600
#define OneBitDelay (1000000/Baudrate)
#define DataBitCount 8
    
    void uartWritePacket(uint8_t *packet);
    
    void uartWriteByte(uint8_t dat);
    
    void craftPacket(uint8_t x, uint8_t y, uint8_t *packet);
    
    void initUart(void);


#ifdef	__cplusplus
}
#endif

#endif	/* POLOLU_CMD_H */

