// comms-mcu.c
// RS485 communications MCU using the PIC18F16Q41. 
//
// Peter J.
// 2023-03-16 basic interpreter from 2023 notes and demo codes.
// 2023-03-17 For interacting with AVR-MCU: Event#, Busy# and Restart#.
//
// CONFIG1
#pragma config FEXTOSC = OFF
#pragma config RSTOSC = HFINTOSC_64MHZ

// CONFIG2
#pragma config CLKOUTEN = OFF
#pragma config PR1WAY = OFF
#pragma config CSWEN = OFF
#pragma config FCMEN = OFF
#pragma config FCMENP = OFF
#pragma config FCMENS = OFF

// CONFIG3
#pragma config MCLRE = EXTMCLR
#pragma config PWRTS = PWRT_64
#pragma config MVECEN = OFF
#pragma config IVT1WAY = OFF
#pragma config LPBOREN = OFF
#pragma config BOREN = SBORDIS

// CONFIG4
#pragma config BORV = VBOR_1P9
#pragma config ZCD = OFF
#pragma config PPS1WAY = OFF
#pragma config STVREN = ON
#pragma config LVP = ON
#pragma config XINST = OFF

// CONFIG5
#pragma config WDTCPS = WDTCPS_31
#pragma config WDTE = ON

// CONFIG6
#pragma config WDTCWS = WDTCWS_7
#pragma config WDTCCS = SC

// CONFIG7
#pragma config BBSIZE = BBSIZE_512
#pragma config BBEN = OFF
#pragma config SAFEN = OFF
#pragma config DEBUG = OFF

// CONFIG8
#pragma config WRTB = OFF
#pragma config WRTC = OFF
#pragma config WRTD = OFF
#pragma config WRTSAF = OFF
#pragma config WRTAPP = OFF

// CONFIG9
#pragma config CP = OFF

#include <xc.h>
#include "global_defs.h"
#include <stdint.h>
#include <stdlib.h>

#include "uart.h"
#include <stdio.h>
#include <string.h>

#define VERSION_STR "v0.4 AVR-eDAQS node 2023-03-18"

// Each device on the RS485 network has a unique single-character identity.
// The master (PC) has identity '0'. Slave nodes may be 1-9A-Za-z.
// When programming each device, select a suitable value for MYID.
#define MYID '1'

#define GREENLED (LATCbits.LATC4)
#define READYPIN (PORTCbits.RC7)
#define EVENTPIN (PORTBbits.RB7)
#define RESTARTn (LATCbits.LATC6)

void init_pins()
{
    // RC4 as digital-output for GREENLED.
    GREENLED = 0;
    TRISCbits.TRISC4 = 0;
    ANSELCbits.ANSELC4 = 0;
    //
    // RC7 as digital-input for DAQ-MCU Ready/Busy# signal.
    TRISCbits.TRISC7 = 1;
    ANSELCbits.ANSELC7 = 0;
    //
    // RB7 as digital-input for Event# signal.
    TRISBbits.TRISB7 = 1;
    ANSELBbits.ANSELB7 = 0;
    //
    // RC6 as digital-output for restart of DAQ_MCU
    ODCONCbits.ODCC6 = 1;
    RESTARTn = 1;
    TRISCbits.TRISC6 = 0;
    ANSELCbits.ANSELC6 = 0;
    //
    // RC3 as analog-in for external trigger.
    //
    return;
}

#define NBUFA 80 
char bufA[NBUFA]; // for incoming RS485 comms
#define NBUFB 132
char bufB[NBUFB]; // for outgoing RS485 comms
#define NBUFC 32
char bufC[NBUFC]; // for outgoing AVR commands
#define NBUFD 132
char bufD[NBUFD]; // for incoming AVR responses

int find_char(char* buf, int start, int end, char c)
// Returns the index of the character if found, -1 otherwise.
// start is the index of the first character to check.
// end is the index of the last character to check.
{
    for (int i = start; i <= end; i++) {
        if (buf[i] == '\0') return -1;
        if (buf[i] == c) return i;
    }
    return -1;
}

char* trim_command(char* buf, int nbytes)
// Returns a pointer to the command text string, within buf.
// The resulting string may be zero-length.
//
// A valid incoming command from the RS485 will be of the form
// "/cXXXXXXXX!"
// where the components are
//    / is the start character
//    ! is the end character
//    c is the MYID character, identifying the receiving node
//    XXXXXXX is the command text
//
// This format is described in the text:
// J.M. Hughes
// Real World Instrumentation
// O'Rielly 2010
// Chapter 11 Instrumentation Data I/O, Unique Protocols.
// 
{
    // printf("DEBUG: buf=%s", buf);
    int start = find_char(buf, 0, nbytes-1, '/');
    if (start == -1) {
        // Did not find starting '/'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    int end = find_char(buf, start, nbytes-1, '!');
    if (end == -1) {
        // Did not find terminating '!'
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, we have a valid incoming command.
    if (buf[start+1] != MYID) {
        // The incoming message is not for this node, so discard it.
        buf[0] = '\0'; // make it a zero-length string
        return &buf[0];
    }
    // At this point, the message is for us.
    buf[end] = '\0'; // Trim off the '!' character.
    // On return, omit the MYID character from the front.
    return &buf[start+2];
} // end trim_command()

void interpret_command(char* cmdStr)
// We intend that valid commands are answered quickly
// so that the supervisory PC can infer the absence of a node
// by the lack of a prompt response.
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    switch (cmdStr[0]) {
        case 'v':
            nchar = snprintf(bufB, NBUFB, "/0 %s#", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 'Q':
            nchar = snprintf(bufB, NBUFB, "/0Q %d %d#", EVENTPIN, READYPIN);
            uart1_putstr(bufB);
            break;
        case 'R':
            RESTARTn = 0;
            __delay_ms(1);
            RESTARTn = 1;
            nchar = snprintf(bufB, NBUFB, "/0R DAQ_MCU restarted#");
            uart1_putstr(bufB);
            break;
        case 'L':
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume on/off value.
                // Use just the least-significant bit.
                i = (uint8_t) (atoi(token_ptr) & 1);
                GREENLED = i;
                nchar = snprintf(bufB, NBUFB, "/0L %d#", i);
            } else {
                // There was no text to give a value.
                nchar = snprintf(bufB, NBUFB, "/0L error: no value#");
            }
            uart1_putstr(bufB);
            break;
        default:
           ; // Do nothing.
    }
} // end interpret_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200);
    __delay_ms(10);
    // We will operate the MCU as a slave, 
    // waiting for commands and only responding when spoken to.
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // CR signals end of incoming string.
        m = uart1_getstr(bufA, NBUFA);
        if (m > 0) {
            char* cmd = trim_command(bufA, NBUFA);
            interpret_command(cmd);
        }
    }
    uart1_flush_rx();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
