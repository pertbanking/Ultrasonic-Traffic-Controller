
#include "pololu_cmd.h"


void uartWritePacket(uint8_t *packet) {
    for(uint8_t i = 0; i < PACKET_SIZE; ++i) {
        uartWriteByte(packet[i]);
    }
}

void uartWriteByte(uint8_t dat) {
    TX = 0;
    __delay_us(OneBitDelay);
    for(uint8_t b = 0; b < DataBitCount; ++b) {
        if(((dat >> b) & 0x01) == 0x01) {
            TX = 1;
        } else {
            TX = 0;
        }
        __delay_us(OneBitDelay);
    }
    TX = 1;
    __delay_us(OneBitDelay);
}
    
void craftPacket(uint8_t x, uint8_t y, uint8_t *packet) {
    packet[0] = PACKET_START;
    packet[3] = PACKET_END;
    if(x > MAX_SPEED) packet[1] = MAX_SPEED;
    else packet[1] = x;
    if(y > MAX_SPEED) packet[2] = MAX_SPEED;
    else packet[2] = y;
}

void initUart(void) {
    BIT_BANG_DIR = 0; // set to output
    TX = 1;
}
