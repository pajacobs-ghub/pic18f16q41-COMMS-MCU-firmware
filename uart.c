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
    // We are going to use hardware control for the RS485
    // but we are going to bypass CTSn/RTSn.
    unsigned int brg_value;
    //
    // Configure PPS RX1=RC1, TX1=RC0, TX1DE=RA2 
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    U1RXPPS = 0b010001; // RC1
    U1CTSPPS = 0b001011; // RB3 does not exist and should always read as 0
    RC0PPS = 0x10; // UART1 TX
    RA2PPS = 0x11; // UART1 TXDE
    // Do not assign UART1RTS to any output pin.
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
// stopping when we see a new-line character.
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
        if (c == '\n') {
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

//----------------------------------------------------------------------------
// UART2 is used to communicate with AVR DAQ-MCU.

void uart2_init(long baud)
{
    // Follow recipe given in PIC18F16Q41 data sheet
    // Sections 34.2.1.8 and 34.2.2.1
    // We do not use hardware flow control.
    unsigned int brg_value;
    //
    // Configure PPS RX2=RB5, TX2=RB4 
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    U2RXPPS = 0b001101; // RB5
    RB4PPS = 0x13; // UART2 TX
    U2CTSPPS = 0b001010; // RB2 does not exist and should always read as 0
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    ANSELBbits.ANSELB4 = 0; // TX2 on RB4 pin
    TRISBbits.TRISB4 = 0; // output
    ANSELBbits.ANSELB5 = 0; // Turn on digital input buffer for RX2
    TRISBbits.TRISB5 = 1; // RX2 on RB5 is an input
    //
    U2CON0bits.BRGS = 1;
    brg_value = (unsigned int) (FOSC/baud/4 - 1);
    // For 64MHz,   9600 baud                 1665.  (error 0.04%)
    //            115200 baud, expect value of 137.  (error 0.6%)
    //            230400 baud                   68.  (error 0.6%)
    //            460800 baud                   33.  (error 2.2%)
    U2BRG = brg_value;
    //
    U2CON0bits.MODE = 0b0000; // Use 8N1 asynchronous
    U2CON2bits.FLO = 0b00; // Hardware flow control off
    U2CON0bits.RXEN = 1;
    U2CON0bits.TXEN = 1;
    U2CON1bits.ON = 1;
    return;
}

void uart2_putch(char data)
{
    // Wait until shift-register empty, then send data.
    while (!U2ERRIRbits.TXMTIF) { CLRWDT(); }
    U2TXB = data;
    return;
}

void uart2_putstr(char* str)
{
    for (size_t i=0; i < strlen(str); i++) uart2_putch(str[i]); 
    return;
}

void uart2_flush_rx(void)
{
    U2FIFObits.RXBE = 1;
}

char uart2_getch(void)
{
    char c;
    // Block until a character is available in buffer.
    while (U2FIFObits.RXBE) { CLRWDT(); }
    // Get the data that came in.
    c = U2RXB;
    return c;
}

int uart2_getstr(char* buf, int nbuf)
// Read (without echo) a line of characters into the buffer,
// stopping when we see a new-line character.
// Returns the number of characters collected,
// excluding the terminating null char.
{
    int i = 0;
    char c;
    uint8_t done = 0;
    while (!done) {
        c = uart2_getch();
        if (c != '\n' && c != '\r' && c != '\b' && i < (nbuf-1)) {
            // Append a normal character.
            buf[i] = c;
            i++;
        }
        if (c == '\n') {
            // Stop collecting on receiving a new-line character.
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

void uart2_close(void)
{
    U2CON0bits.RXEN = 1;
    U2CON0bits.TXEN = 1;
    U2CON1bits.ON = 1;
    return;
}
