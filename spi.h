#ifndef SPI_H
#define SPI_H

#include <avr/io.h>
#include <avr/interrupt.h>

void SPI_Master_Init(void){
    DDRB |= 0xA0; PORTB |= 0x40;// set sck, mosi to output and miso to input
    // Set SPCR register to enable SPI, enable master, and use SCK frequency
    //   of fosc/16
    SPCR = (1 << SPE) | (1 << MSTR);
    SPSR |= (1 << SPI2X);
    // Make sure global interrupts are enabled on SREG register
    SREG |= 0x80;
}

unsigned char SPI_MasterTransmit(unsigned char cData){
    //SS controlled by user
    // data in SPDR will be transmitted, e.g. SPDR = cData;
    SPDR = cData;
    while(!(SPSR & (1 << SPIF))) { // wait for transmission to complete
        ;
    }
    return SPDR;
}

#endif