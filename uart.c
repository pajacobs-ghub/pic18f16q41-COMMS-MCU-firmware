// uart.c
// Functions to provide a shim between the C standard library functions
// and UART1 peripheral device on the PIC16F1/PIC18F microcontroller.
// Build with linker set to link in the C99 standard library.
// PJ,
// 2023-12-01 PIC18F16Q41 attached to a MAX3082 RS485 transceiver

#include <xc.h>
#include "global_defs.h"
#include "uart.h"
#include <stdio.h>
#include <string.h>

void uart1_init(long baud)
{
    // Follow recipe given in PIC18F16Q41 data sheet
    // Sections 34.2.1.8 and 34.2.2.1
    unsigned int brg_value;
    //
    // Configure PPS RX1=RC1, TX1=RC0, TX1DE=RA2 
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    U1RXPPS = 0b010001; // RC1
    RC0PPS = 0x10; // UART1 TX
    RA2PPS = 0x11; // UART1 TXDE
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    ANSELCbits.ANSELC0 = 0; // TX pin
    TRISCbits.TRISC0 = 0; // output
    ANSELAbits.ANSELA2 = 0; // TXDE pin
    TRISAbits.TRISA2 = 0; // output
    ANSELCbits.ANSELC1 = 0; // Turn on digital input buffer for RX1
    TRISCbits.TRISC1 = 1; // RX1 is an input
    //
    U1CON0bits.BRGS = 1;
    brg_value = (unsigned int) (FOSC/baud/4 - 1);
    // For 64MHz, 115200 baud, expect value of 137.
    //              9600 baud                 1665.
    U1BRG = brg_value;
    //
    U1CON0bits.MODE = 0b0000; // Use 8N1 asynchronous
    U1CON2bits.FLO = 0b10; // Hardware flow control (TXDE)
    U1CON0bits.RXEN = 1;
    U1CON0bits.TXEN = 1;
    U1CON1bits.ON = 1;
    return;
}

void uart1_putch(char data)
{
    // Wait until shift-register empty, then send data.
    while (!U1ERRIRbits.TXMTIF) { CLRWDT(); }
    U1TXB = data;
    return;
}

void uart1_putstr(char* str)
{
    for (size_t i=0; i < strlen(str); i++) uart1_putch(str[i]); 
    return;
}

void uart1_flush_rx(void)
{
    U1FIFObits.RXBE = 1;
}

char uart1_getch(void)
{
    char c;
    // Block until a character is available in buffer.
    while (U1FIFObits.RXBE) { CLRWDT(); }
    // Get the data that came in.
    c = U1RXB;
    return c;
}

int uart1_getstr(char* buf, int nbuf)
// Read (without echo) a line of characters into the buffer,
// stopping when we see a return character.
// Returns the number of characters collected,
// excluding the terminating null char.
{
    int i = 0;
    char c;
    uint8_t done = 0;
    while (!done) {
        c = uart1_getch();
        if (c != '\n' && c != '\r' && c != '\b' && i < (nbuf-1)) {
            // Append a normal character.
            buf[i] = c;
            i++;
        }
        if (c == '\r') {
            // Stop collecting on receiving a carriage-return character.
            done = 1;
            buf[i] = '\0';
        }
        if (c == '\b' && i > 0) {
            // Backspace.
            i--;
        }
    }
    return i;
}

void uart1_close(void)
{
    U1CON0bits.RXEN = 1;
    U1CON0bits.TXEN = 1;
    U1CON1bits.ON = 1;
    return;
}

// Functions to connect STDIO to UART1.

void putch(char data)
{
    uart1_putch(data);
    return;
}

int getch(void)
{
    return uart1_getch();
}

int getche(void)
{
    int data = getch();
    putch((char)data); // echo the character
    return data;
}
