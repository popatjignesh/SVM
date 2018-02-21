/* 
 * File:   LedBlink.c
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
#pragma config BODENV = BORV45          // Brown Out Voltage (4.5V)
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

#define FCY 2000000UL       // instruction cycle Hz

#define __delay_ms(d) \
  { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000ULL)); }
#define __delay_us(d) \
  { __delay32( (unsigned long) (((unsigned long long) d)*(FCY)/1000000ULL)); }

#define LED LATDbits.LATD1      //LED Output PIN

void InitIO(void);

void main()
{
    InitIO();
    Nop();
	
	LED = 0;
    while(1);
    
	while(1)
    {
		LED = ~LED;
		__delay_ms(1000);
        __delay_ms(1000);
        __delay_ms(1000);
        __delay_ms(1000);
	}
}


void InitIO(void)
{
    ADPCFG = 0x0000;        //Set all pins as Digital

	TRISB = 1;              //Set all pins as Output
	TRISC = 1;
	TRISD = 1;
	TRISE = 1;
	TRISF = 1;
    
    TRISDbits.TRISD1 = 0;   //Define LED as an output

	LATB = 0;               //all pins output 0 value
	LATC = 0;
	LATD = 0;
	LATE = 0;
    LATF = 0;
}
