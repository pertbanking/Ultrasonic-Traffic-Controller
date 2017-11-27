/*
 * File:   main.c
 * Author: Liam
 *
 * Created on October 30, 2017, 12:42 PM
 */


// CONFIG1 
 #pragma config FOSC = INTOSC    // Oscillator Selection Bits (INTOSC oscillator: I/O function on CLKIN pin) 
 #pragma config WDTE = OFF       // Watchdog Timer Enable (WDT disabled) 
 #pragma config PWRTE = OFF      // Power-up Timer Enable (PWRT disabled) 
 #pragma config MCLRE = ON       // MCLR Pin Function Select (MCLR/VPP pin function is MCLR) 
 #pragma config CP = OFF         // Flash Program Memory Code Protection (Program memory code protection is disabled) 
 #pragma config BOREN = OFF      // Brown-out Reset Enable (Brown-out Reset disabled) 
 #pragma config CLKOUTEN = OFF   // Clock Out Enable (CLKOUT function is disabled. I/O or oscillator function on the CLKOUT pin) 
 
 // CONFIG2 
 #pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off) 
 #pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable (Stack Overflow or Underflow will cause a Reset) 
 #pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.) 
 #pragma config LPBOR = OFF      // Low-Power Brown Out Reset (Low-Power BOR is disabled) 
 #pragma config LVP = ON        // Low-Voltage Programming Enable (High-voltage on MCLR/VPP must be used for programming) 
 
#define _XTAL_FREQ 16000000UL
#define ULTRASONIC_RECEIVER PORTAbits.RA4
#define PAYLOAD_LENGTH 3
#define IRQ_TIMEOUT 50000


#include <xc.h>
#include "spi.h"
#include <stdint.h>
#include "NRF_consts.h"

const uint8_t TOWER_ID = 0xb1;
const uint8_t START_TIMING = 20;
const int TIMEOUT = 80000;

void configureTX(void);
int transmitData(uint8_t *data, uint8_t len);
void configureReceiver(void);
void readData(uint8_t *buf);
void configureRX(void);

int main(void) {
    
    TRISAbits.TRISA1 = 0;
    LATAbits.LATA1 = 0;
    OSCCONbits.IRCF = 0xF;
    
    
    configureReceiver();
    SPI_init();
    configureTX();
    
    uint8_t test[3];
    test[0] = TOWER_ID;
    test[1] = 0xdd;
    test[2] = 0xee;
  
    // LATAbits.LATA1 = 1;
    configureRX();
   
  
    uint8_t buf[PAYLOAD_LENGTH];
    
    while(1) {
        while(SPI_read_byte(0x00) != 0b00111011) configureRX();
        NRF_CE = 1;
        LATAbits.LATA1 = 0;
        int irqTime = 0;
        while(IRQ) {
            /*
            ++irqTime;
            if(irqTime == IRQ_TIMEOUT) {
                NRF_CE = 0;
                __delay_us(10);
                NRF_CE = 1;
                irqTime = 0;
            }
             * */
        }
        NRF_CE = 0;
        readData(buf);
        if(buf[0] == START_TIMING) {
            LATAbits.LATA1 = 1;
            // 0xaa is start timing command (MAKE THIS A CONSTANT)
            int timeCount = 0;
            while(ULTRASONIC_RECEIVER) {
                timeCount++;
            }
            
            configureTX();
            uint8_t distancePacket[3];
            distancePacket[0] = TOWER_ID;
            distancePacket[1] = (timeCount & 0xff);
            distancePacket[2] = (timeCount >> 8) & 0xff;
            transmitData(distancePacket, 3);
            
            /*
            int timeCount = 0;
            
            while((ULTRASONIC_RECEIVER) && (timeCount < TIMEOUT)) {
                ++timeCount;
            }
            if(timeCount != TIMEOUT) {
                // LATAbits.LATA1 = !LATAbits.LATA1;
                configureTX();
                uint8_t distancePacket[3];
                distancePacket[0] = TOWER_ID;
                distancePacket[1] = (timeCount & 0xff);
                distancePacket[2] = (timeCount >> 8) & 0xff;
                transmitData(distancePacket, 3);
                // while(SPI_read_byte(0x00) != 0b00111011) configureRX();
                // configureRX();
            }
             * */
        }
    }
    
    return 0;
}


void configureTX(void) {
    // 1. need to pull PRIM_RX bit low
    // 2. clock in address for receiving node (TX_ADDR) and payload data
    // (TX_PLD) via SPI. Must write TX_PLD continuously while holding CSN low.
    // TX_ADDR does not have to be rewritten if unchanged from last transmit!
    // If PTX device shall receive acknowledge, data pipe 0 has to be configured
    // to receive the acknowledgment
    // 3. A high pulse of CE starts the transmission. Minimum pulse width of
    // 10 microseconds
    // 4. stuff happens
    // 5. auto acknoledgment happens and TX_DS in the status register is
    // set high. IRQ will be active when TX_DS or MAX_RT is set high. to turn
    // of IRQ, interrupt source must be reset by writing to SREG
    NRF_CE = 0;
    __delay_ms(1);
    SS_PIN = 1;
    __delay_ms(1);
    uint8_t write[2];
    write[0] = (CONFIG & REGISTER_MASK) | W_MASK;
    write[1] = 0b01001010;  // config stuff. show max_rt and tx_ds
    // interrupts. mask rx_dr interrupt. enable crc, crc 1 byte,
    // power up, ptx mode
    SPI_writeArray(write, 2);
    
   
    write[0] = (EN_AA & REGISTER_MASK) | W_MASK;
    write[1] = 0x01;
    SPI_writeArray(write, 2); // enable auto-acknowledgement on pipe 0
    
    write[0] = (EN_RX_ADDR & REGISTER_MASK) | W_MASK;
    write[1] = 0b00000001; // enable data pipe 0
    SPI_writeArray(write, 2);
    
    write[0] = (RX_PW_P[0] & REGISTER_MASK) | W_MASK;
    write[1] = PAYLOAD_LENGTH;
    SPI_writeArray(write, 2); // multibyte in RX payload in data pipe 0
    
    write[0] = (SETUP_AW & REGISTER_MASK) | W_MASK;
    write[1] = 0b0000011;
    SPI_writeArray(write, 2);
}



int transmitData(uint8_t *data, uint8_t len) {
    uint8_t write[11]; // 10 is max len
    write[0]= W_TX_PAYLOAD; //command
    for(uint8_t i = 0; i < len; ++i) {
        write[1+i] = data[i];// data
    }
    SPI_writeArray(write, 1 + len);
    uint8_t sreg;
    do {
        NRF_CE = 1;
        __delay_us(15);
        NRF_CE = 0;
        while(IRQ);
        SS_PIN = 0;
        sreg = SPI_write_byte(GET_SREG);
        SS_PIN = 1;
        resetIRQ();
    } while(!(sreg & 0b00110000)); // check to see if IRQ was
    // asserted due to auto acknowledgment
    
    return (sreg & 0b00010000); // returns non-zero for error
    
}

void configureReceiver(void) {
    TRISAbits.TRISA4 = 1; // input for receiver
}

void readData(uint8_t *buf) {
    SS_PIN = 0;
    SPI_write_byte(R_RX_PAYLOAD);
    for(uint8_t i = 0; i < PAYLOAD_LENGTH; ++i) {
        buf[i] = SPI_write_byte(0xff); // dummy
    }
    SS_PIN = 1;
}

void configureRX(void) {
    // 1. need to pull PRIM_RX bit low
    // 2. clock in address for receiving node (TX_ADDR) and payload data
    // (TX_PLD) via SPI. Must write TX_PLD continuously while holding CSN low.
    // TX_ADDR does not have to be rewritten if unchanged from last transmit!
    // If PTX device shall receive acknowledge, data pipe 0 has to be configured
    // to receive the acknowledgment
    // 3. A high pulse of CE starts the transmission. Minimum pulse width of
    // 10 microseconds
    NRF_CE = 0;
    __delay_ms(1);
    SS_PIN = 1;
    __delay_ms(1);
    uint8_t write[2];
    write[0] = (CONFIG & REGISTER_MASK) | W_MASK;
    write[1] = 0b00111011;  // config stuff, PRX, power up
    SPI_writeArray(write, 2);
    
   
    write[0] = (EN_AA & REGISTER_MASK) | W_MASK;
    write[1] = 0b00000001;
    SPI_writeArray(write, 2); // enable auto-acknowledgement on pipe 0
    
    write[0] = (EN_RX_ADDR & REGISTER_MASK) | W_MASK;
    write[1] = 0b00000001; // enable data pipe 0
    SPI_writeArray(write, 2);
    
    write[0] = (RX_PW_P[0] & REGISTER_MASK) | W_MASK;
    write[1] = PAYLOAD_LENGTH;
    SPI_writeArray(write, 2); // multibyte in RX payload in data pipe 0
    
    write[0] = (SETUP_AW & REGISTER_MASK) | W_MASK;
    write[1] = 0b0000011;
    SPI_writeArray(write, 2);
    
}