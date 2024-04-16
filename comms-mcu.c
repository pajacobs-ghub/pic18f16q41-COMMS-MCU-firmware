// comms-mcu.c
// RS485 communications MCU using the PIC18F16Q41.
//
// Peter J.
// 2023-03-16 basic interpreter from 2023 notes and demo codes.
// 2023-03-17 For interacting with AVR-MCU: Event#, Busy# and Restart#.
// 2024-03-29 Have pass-through commands working.
// 2024-04-02 Implement software trigger (to assert Event# line).
// 2024-04-06 Set VREF (on OPA1OUT pin) on and off; implement external trigger.
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

#define VERSION_STR "v0.14 PIC18F16Q41 COMMS-MCU 2024-04-09"

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
    // Later, it may be driven by this MCU, on software command.
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

static inline void assert_event_pin()
{
    // Actively pull the event pin low.
    // Of course, this will not work if the CLC1 has control of RB7.
    LATBbits.LATB7 = 0;
    ODCONBbits.ODCB7 = 1;
    TRISBbits.TRISB7 = 0;
}

static inline void release_event_pin()
{
    // Release the drive on the pin and allow it to be pulled high.
    // Should not be used if CLC1 has control of RB7.
    LATBbits.LATB7 = 1;
    TRISBbits.TRISB7 = 1;
}

void FVR_init()
{
    // We want to supply both the ADC and the DAC with 4V.
    FVRCONbits.ADFVR = 3;  // 4v096
    FVRCONbits.CDAFVR = 3; // 4v096
    FVRCONbits.EN = 1;
    while (!FVRCONbits.RDY) { /* should be less than 25 microseconds */ }
    return;
}

void FVR_close()
{
    FVRCONbits.EN = 0;
    FVRCONbits.ADFVR = 0;
    FVRCONbits.CDAFVR = 0;
    return;
}

void set_VREF_on(uint8_t level)
{
    // Assuming that the fixed voltage reference is on at 4v096,
    // take a fraction of that voltage and feed it through the
    // DAC1 and then through the OPA1 to the external pin (OPA1OUT/RC2).
    //
    DAC1CONbits.PSS = 0b10; // FVR Buffer 2
    DAC1CONbits.NSS = 0; // VSS
    DAC1CONbits.EN = 1;
    DAC1DATL = level;
    //
    OPA1CON2bits.PCH = 0b100; // DAC1_OUT
    OPA1CON0bits.UG = 1; // unity gain
    OPA1CON0bits.CPON = 1; // charge pump active
    OPA1CON0bits.SOC = 0; // basic operation
    OPA1CON0bits.EN = 1;
    return;
}

void set_VREF_off()
{
    OPA1CON0bits.EN = 0;
    DAC1CONbits.EN = 0;
    return;
}

void ADC_init()
{
    // Set up the ADC to look at the comparator input pin C1IN3/RC3.
    TRISCbits.TRISC3 = 1;
    ANSELCbits.ANSELC3 = 1;
    //
    ADCON0bits.CS = 1; // use dedicated RC oscillator
    ADCON0bits.FM = 1; // right-justified result
    ADCON2bits.ADMD = 0b000; // basic (legacy) behaviour
    PIR1bits.ADIF = 0;
    ADREFbits.NREF = 0; // negative reference is Vss
    ADREFbits.PREF = 0b11; // positive reference is FVR
    ADACQ = 0x10; // 16TAD acquisition period
    ADPCH = 0x13; // select RC3/ANC3
    ADCON0bits.ON = 1; // power on the device

    return;
}

uint16_t ADC_read()
{
    // Returns the value from the ADC when looking at the input pin
    // for the comparator.
    ADCON0bits.GO = 1;
    NOP();
    while (ADCON0bits.GO) { /* wait, should be brief */ }
    PIR1bits.ADIF = 0;
    return ADRES;
}

void ADC_close()
{
    ADCON0bits.ON = 0;
    return;
}

uint8_t enable_comparator(uint8_t level, int8_t slope)
{
    // Input:
    // level sets the trigger voltage
    // slope=1 to trigger on exceeding level
    // slope=0 to trigger on going below level
    //
    // Returns:
    // 0 if successfully set up,
    // 1 if the comparator is already high.
    //
    // Use DAC2 for the reference level.
    DAC2CONbits.PSS = 0b10; // FVR Buffer 2
    DAC2CONbits.NSS = 0; // VSS
    DAC2CONbits.EN = 1;
    DAC2DATL = level;
    __delay_ms(1);
    //
    // Our external signal goes into the inverting input of the comparator,
    // so we need to invert the polarity to get trigger on positive slope
    // of the external signal.
    CM1NCH = 0b011; // C1IN3- pin
    CM1PCH = 0b101; // DAC2_Output
    if (slope) {
        CM1CON0bits.POL = 1;
    } else {
        CM1CON0bits.POL = 0;
    }
    CM1CON0bits.HYS = 0; // no hysteresis
    CM1CON0bits.SYNC = 0; // async output
    CM1CON0bits.EN = 1;
    // The signal out of the comparator should transition 0 to 1
    // as the external trigger voltage crosses the specified level.
    __delay_ms(1);
    if (CMOUTbits.MC1OUT) {
        // Fail early because the comparator is already triggered.
        return 1;
    }
    // Use CLC1 to latch the comparator output.
    //
    // Follow the set-up description in Section 22.6 of datasheet.
    CLCSELECT = 0b00; // To select CLC1 registers for the following settings.
    CLCnCONbits.EN = 0; // Disable while setting up.
    // Data select from outside world
    CLCnSEL0 = 0b00011100; // data1 gets CMP1_OUT as input
    CLCnSEL1 = 0; // data2 gets CLCIN0PPS as input, but gets ignored in logic select
    CLCnSEL2 = 0; // data3 as for data2
    CLCnSEL3 = 0; // data4 as for data2
    // Logic select into gates
    CLCnGLS0 = 0b10; // data1 goes through true to gate 1 (S-R set)
    CLCnGLS1 = 0; // gate 2 gets logic 0 (S-R set)
    CLCnGLS2 = 0; // gate 3 gets logic 0 (S-R reset)
    CLCnGLS3 = 0; // gate 4 gets logic 0 (S-R reset)
    // Gate output polarities
    CLCnPOLbits.G1POL = 0;
    CLCnPOLbits.G2POL = 0;
    CLCnPOLbits.G3POL = 0;
    CLCnPOLbits.G4POL = 0;
    // Logic function is S-R latch
    CLCnCONbits.MODE = 0b011;
    // Invert the CLC output because we want active low EVENT# signal
    CLCnPOLbits.POL = 1;
    // Connect the output of the CLC1 to the EVENTn pin.
    ODCONBbits.ODCB7 = 1; // Open-drain output
    TRISBbits.TRISB7 = 0; // Drive as an output.
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RB7PPS = 0x01; // CLC1OUT
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    // Now that the S-R latch is set up, enable it.
    CLCnCONbits.EN = 1;
    return 0; // Success is presumed.
}

void disable_comparator()
{
    // Release the EVENTn pin and disable the comparator and CLC.
    LATBbits.LATB7 = 1;
    TRISBbits.TRISB7 = 1; // return to being an input
    GIE = 0;
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 0;
    RB7PPS = 0x00; // LATB7 (back to software control)
    PPSLOCK = 0x55;
    PPSLOCK = 0xaa;
    PPSLOCKED = 1;
    //
    CLCSELECT = 0b00; // To select CLC1 for the following setting.
    CLCnCONbits.EN = 0;
    CM1CON0bits.EN = 0;
    DAC2CONbits.EN = 0;
    return;
}

// For incoming RS485 comms
#define NBUFA 80
char bufA[NBUFA];
// For outgoing RS485 comms
#define NBUFB 268
char bufB[NBUFB];
// For incoming AVR responses
#define NBUFC 256
char bufC[NBUFC];

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

char* trim_RS485_command(char* buf, int nbytes)
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

void interpret_RS485_command(char* cmdStr)
// We intend that valid commands are answered quickly
// so that the supervisory PC can infer the absence of a node
// by the lack of a prompt response.
// A command that does not do what is expected should return a message
// that includes the word "error".
{
    char* token_ptr;
    const char* sep_tok = ", ";
    int nchar;
    uint8_t i, j;
    // nchar = printf("DEBUG: cmdStr=%s", cmdStr);
    switch (cmdStr[0]) {
        case 'v':
            nchar = snprintf(bufB, NBUFB, "/0v %s#\n", VERSION_STR);
            uart1_putstr(bufB);
            break;
        case 't':
            // Software trigger to assert EVENTn line low.
            assert_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0t Software trigger#\n");
            uart1_putstr(bufB);
            break;
        case 'z':
            // Release EVENTn line (from software trigger).
            release_event_pin();
            nchar = snprintf(bufB, NBUFB, "/0z Release EVENTn line#\n");
            uart1_putstr(bufB);
            break;
        case 'Q':
            // Query the status signals.
            // READY/BUSYn is from the AVR.
            // EVENTn is a party line that anyone may pull low.
            nchar = snprintf(bufB, NBUFB, "/0Q %d %d#\n", EVENTPIN, READYPIN);
            uart1_putstr(bufB);
            break;
        case 'F':
            // Flush the RX2 buffer for incoming text from the AVR.
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0F Flushed RX2 buffer#\n");
            uart1_putstr(bufB);
            break;
        case 'R':
            // Restart the attached AVR MCU.
            RESTARTn = 0;
            __delay_ms(1);
            RESTARTn = 1;
            // Wait until we are reasonably sure that the AVR has restarted
            // and then flush the incoming serial buffer.
            __delay_ms(350);
            uart2_flush_rx();
            nchar = snprintf(bufB, NBUFB, "/0R DAQ_MCU restarted#\n");
            uart1_putstr(bufB);
            break;
        case 'L':
            // Turn LED on or off.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume on/off value.
                // Use just the least-significant bit.
                i = (uint8_t) (atoi(token_ptr) & 1);
                GREENLED = i;
                nchar = snprintf(bufB, NBUFB, "/0L %d#\n", i);
            } else {
                // There was no text to give a value.
                nchar = snprintf(bufB, NBUFB, "/0L error: no value#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'a': {
            // Report the ADC value for the analog signal on the comparator input.
            uint16_t aValue = ADC_read();
            nchar = snprintf(bufB, NBUFB, "/0a %u#\n", aValue);
            uart1_putstr(bufB); }
            break;
        case 'e':
            // Enable comparator, to pull EVENT# line low on the
            // external trigger signal.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume trigger level
                // and, maybe, slope flag.
                int16_t level = atoi(token_ptr);
                if (level > 255) level = 255;
                if (level < 0) level = 0;
                int8_t slope = 1;
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    slope = (int8_t) atoi(token_ptr);
                }
                uint8_t flag = enable_comparator((uint8_t)level, slope);
                if (flag) {
                    nchar = snprintf(bufB, NBUFB, "/0e error: comparator already triggered#\n");
                } else {
                    nchar = snprintf(bufB, NBUFB, "/0e %d %d#\n", level, slope);
                }
            } else {
                // There was no text to give a trigger level.
                nchar = snprintf(bufB, NBUFB, "/0e error: no level supplied#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'd':
            // Disable comparator and release EVENTn line.
            disable_comparator();
            nchar = snprintf(bufB, NBUFB, "/0d Disable comparator#\n");
            uart1_putstr(bufB);
            break;
        case 'w':
            // Enable VREF output, to feed an external ADC chip on the DAC-MCU.
            // Note that this is not relevant to all ADC MCUs.
            token_ptr = strtok(&cmdStr[1], sep_tok);
            if (token_ptr) {
                // Found some non-blank text; assume level followed by on/off flag.
                // The on/off flag is optional and defaults to 1.
                int16_t level = atoi(token_ptr);
                int8_t onOffFlag = 1;
                token_ptr = strtok(NULL, sep_tok);
                if (token_ptr) {
                    onOffFlag = (int8_t) atoi(token_ptr);
                }
                if (onOffFlag) {
                    if (level > 255) level = 255;
                    if (level < 0) level = 0;
                    set_VREF_on((uint8_t)level);
                    nchar = snprintf(bufB, NBUFB, "/0w VREF on level=%d#\n", level);
                } else {
                    set_VREF_off();
                    nchar = snprintf(bufB, NBUFB, "/0w VREF off#\n");
                }
            } else {
                // There was no text to indicate action.
                nchar = snprintf(bufB, NBUFB, "/0w error: missing level and on/off flag#\n");
            }
            uart1_putstr(bufB);
            break;
        case 'X':
            // Pass through a command to the AVR MCU.
            if (READYPIN) {
                // AVR is ready and waiting for a command.
                uart2_putstr(&cmdStr[1]);
                uart2_putch('\r');
                // The AVR may start its response within 200us,
                // so start listening for that response immediately.
                uart2_getstr(bufC, NBUFC);
                nchar = snprintf(bufB, NBUFB, "/0X %s#\n", &bufC[0]);
            } else {
                // AVR is not ready for a command.
                nchar = snprintf(bufB, NBUFB, "/0X error: AVR busy#\n");
            }
            uart1_putstr(bufB);
            break;
        default:
            nchar = snprintf(bufB, NBUFB, "/0%c error: Unknown command#\n", cmdStr[0]);
            uart1_putstr(bufB);
    }
} // end interpret_command()

int main(void)
{
    int m;
    int n;
    init_pins();
    uart1_init(115200); // RS485 comms
    uart2_init(230400); // comms to AVR DAQ-MCU
    FVR_init();
    ADC_init();
    __delay_ms(10);
    // Flash LED twice at start-up to indicate that the MCU is ready.
    for (int8_t i=0; i < 2; ++i) {
        GREENLED = 1;
        __delay_ms(250);
        GREENLED = 0;
        __delay_ms(250);
    }
    // Wait until we are reasonably sure that the AVR has restarted
    // and then flush the incoming serial buffer.
    __delay_ms(100);
    uart2_flush_rx();
    // We will operate this COMMS_MCU as a slave,
    // waiting for commands and only responding when spoken to.
    while (1) {
        // Characters are not echoed as they are typed.
        // Backspace deleting is allowed.
        // CR signals end of incoming string.
        m = uart1_getstr(bufA, NBUFA);
        if (m > 0) {
            char* cmd = trim_RS485_command(bufA, NBUFA);
            interpret_RS485_command(cmd);
        }
    }
    set_VREF_off();
    ADC_close();
    FVR_close();
    uart2_flush_rx();
    uart2_close();
    uart1_flush_rx();
    uart1_close();
    return 0; // Expect that the MCU will reset.
} // end main
