/* 
 * File:   ACIM.c
 * Author: Jignesh D. Popat
 *
 * Created on January 31, 2018, 10:29 PM
 */

// DSPIC30F3011 Configuration Bit Settings
// FOSC
#pragma config FOSFPR = FRC             // Oscillator (Internal Fast RC (No change to Primary Osc Mode bits))
#pragma config FCKSMEN = CSW_FSCM_OFF   // Clock Switching and Monitor (Sw Disabled, Mon Disabled)

// FWDT
#pragma config FWPSB = WDTPSB_16        // WDT Prescaler B (1:16)
#pragma config FWPSA = WDTPSA_512       // WDT Prescaler A (1:512)
#pragma config WDT = WDT_OFF            // Watchdog Timer (Disabled)

// FBORPOR
#pragma config FPWRT = PWRT_OFF         // POR Timer Value (Timer Disabled)
#pragma config BODENV = BORV20          // Brown Out Voltage (Reserved)
#pragma config BOREN = PBOR_OFF         // PBOR Enable (Disabled)
#pragma config LPOL = PWMxL_ACT_LO      // Low-side PWM Output Polarity (Active Low)
#pragma config HPOL = PWMxH_ACT_HI      // High-side PWM Output Polarity (Active High)
#pragma config PWMPIN = RST_IOPIN       // PWM Output Pin Reset (Control with PORT/TRIS regs)
#pragma config MCLRE = MCLR_EN          // Master Clear Enable (Enabled)

// FGS
#pragma config GWRP = GWRP_OFF          // General Code Segment Write Protect (Disabled)
#pragma config GCP = CODE_PROT_OFF      // General Segment Code Protection (Disabled)

// FICD
#pragma config ICS = ICS_PGD            // Comm Channel Select (Use PGC/EMUC and PGD/EMUD)

#include "xc.h"
#include <libpic30.h>
#include <stdio.h>
#include <stdlib.h>

#define FCY 2000000UL           // Instruction Cycle Hz
#define FPWM 20000              // 20 kHz
#define DEADTIME (unsigned int)(0.000002 * FCY)
#define _DES_FREQ 60            // 60 Hz sine wave is required
#define _DELTA_PHASE (unsigned int)(_DES_FREQ * 65536 / FPWM)
#define _120_DEGREES 0x5555
#define _240_DEGREES 0xAAAA
#define _360_DEGREES 0x0000

#define __delay_ms(d) \
  { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000ULL)); }
#define __delay_us(d) \
  { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000ULL)); }

//#define LED LATDbits.LATD1      //LED Output PIN

void InitIO(void);
void InitMCPWM(void);

unsigned int Phase, Delta_Phase, Phase_Offset;
int Multiplier, Result;

signed long sinetable[64]={0, 3263, 6493, 9658, 12728, 15671, 18458, 21062, 
23457, 25618, 27525, 29158, 30502, 31542, 32269, 32675, 32757, 32513, 
31945, 31061, 29867, 28377, 26605, 24568, 22287, 19785, 17086, 14217, 
11207, 8085, 4884, 1633, -1633, -4884, -8085, -11207, -14217, -17086, 
-19785, -22287, -24568, -26605, -28377, -29867, -31061, -31945, -32513, 
-32757, -32675, -32269, -31542, -30502, -29158, -27525, -25618, -23457, 
-21062, -18458, -15671, -12728, -9658, -6493, -3263, 0};


void main()
{
    InitIO();
    Nop();
    InitMCPWM();
    Nop();
    
	while(1)
    {
    Phase += Delta_Phase;           // Accumulate Delta_Phase in Phase variable
    
    Phase_Offset = _360_DEGREES;    // Add proper value to phase offset
    Multiplier = sinetable[(Phase + Phase_Offset) >> 10];   // Take sine info
    asm("MOV _Multiplier, W4");     // Load first multiplier
    asm("MOV _PTPER, W5");          // Load second multiplier
    asm("MOV #_Result, W0");        // Load W0 with the address of Result
    asm("MPY W4*W5, A");            // Perform Fractional multiplication
    asm("SAC A, [W0]");             // Store multiplication result in variable Result
    PDC1 = Result + PTPER;          // Remove negative values of the duty cycle
    
    Phase_Offset = _120_DEGREES;    // Add proper value to phase offset
    Multiplier = sinetable[(Phase + Phase_Offset) >> 10];   // Take sine info    
    asm("MOV _Multiplier, W4");     // Load first multiplier
    asm("MOV _PTPER, W5");          // Load second multiplier
    asm("MOV #_Result, W0");        // Load W0 with the address of Result
    asm("MPY W4*W5, A");            // Perform Fractional multiplication
    asm("SAC A, [W0]");             // Store multiplication result in variable Result    
    PDC2 = Result + PTPER;          // Remove negative values of the duty cycle
    
    Phase_Offset = _240_DEGREES;    // Add proper value to phase offset
    Multiplier = sinetable[(Phase + Phase_Offset) >> 10];   // Take sine info
    asm("MOV _Multiplier, W4");     // Load first multiplier
    asm("MOV _PTPER, W5");          // Load second multiplier
    asm("MOV #_Result, W0");        // Load W0 with the address of Result
    asm("MPY W4*W5, A");            // Perform Fractional multiplication
    asm("SAC A, [W0]");             // Store multiplication result in variable Result
    PDC3 = Result + PTPER;          // Remove negative values of the duty cycle
	}
}

void InitIO(void)
{
    ADPCFG = 0x0000;        //Set all pins as Digital

	TRISB = 0;              //Set all pins as Output
	TRISC = 0;
	TRISD = 0;
	TRISE = 0;
	TRISF = 0;
    
    TRISDbits.TRISD1 = 0;   //Define LED as an output

    TRISEbits.TRISE0 = 0;   //Define PWM1L as an output
    TRISEbits.TRISE1 = 0;   //Define PWM1H as an output
    TRISEbits.TRISE2 = 0;   //Define PWM2L as an output
    TRISEbits.TRISE3 = 0;   //Define PWM2H as an output
    TRISEbits.TRISE4 = 0;   //Define PWM3L as an output
    TRISEbits.TRISE5 = 0;   //Define PWM3H as an output
    
	LATB = 0;               //all pins output 0 value
	LATC = 0;
	LATD = 0;
	LATE = 0;
    LATF = 0;
}

void InitMCPWM(void)
{
    PTPER = (FCY/FPWM - 1) >> 1;    // Compute Period for desired frequency
    OVDCON = 0x0000;                // Disable all PWM outputs.
    DTCON1 = DEADTIME;              // ~2 us of dead time @ 20 MIPS and 1:1 Prescaler
    PWMCON1 = 0x0077;               // Enable PWM output pins and enable complementary mode
    PDC1 = PTPER;                   /* 0 Volts on Phase A. This value corresponds to
                                    50% of duty cycle, which in complementary mode
                                    gives an average of 0 Volts */
    PDC2 = PTPER;                   // 0 Volts on Phase B.
    PDC3 = PTPER;                   // 0 Volts on Phase C.
    IFS2bits.PWMIF = 0;             // Clear PWM Interrupt flag
    IEC2bits.PWMIE = 1;             // Enable PWM Interrupts
    OVDCON = 0x3F00;                // PWM outputs are controller by PWM module
    PTCONbits.PTMOD = 2;            // Center aligned PWM operation
    Phase = 0;                      // Reset Phase Variable
    Delta_Phase = _DELTA_PHASE;     // Initialize Phase increment for 60Hz sine wave
    PTCONbits.PTEN = 1;             // Start PWM
}
