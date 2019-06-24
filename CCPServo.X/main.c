/*
 * Minimal example of driving a servo with the PIC CCP module
 * Copyright (c) 2019 David Rice
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// PIC16F18325 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1
#pragma config FEXTOSC = OFF    // FEXTOSC External Oscillator mode Selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32  // Power-up default value for COSC bits (HFINTOSC (1MHz))
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; I/O or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable (Fail-Safe Clock Monitor is disabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR/VPP pin function is MCLR; Weak pull-up enabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config WDTE = OFF       // Watchdog Timer Enable bits (WDT disabled; SWDTEN is ignored)
#pragma config LPBOREN = OFF    // Low-power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled, SBOREN bit ignored)
#pragma config BORV = LOW       // Brown-out Reset Voltage selection bit (Brown-out voltage (Vbor) set to 2.45V)
#pragma config PPS1WAY = ON     // PPSLOCK bit One-Way Set Enable bit (The PPSLOCK bit can be cleared and set only once; PPS registers remain locked after one clear/set cycle)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a Reset)
#pragma config DEBUG = OFF      // Debugger enable bit (Background debugger disabled)

// CONFIG3
#pragma config WRT = OFF        // User NVM self-write protection bits (Write protection off)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/VPP pin function is MCLR. MCLRE configuration bit is ignored.)

// CONFIG4
#pragma config CP = OFF         // User NVM Program Memory Code Protection bit (User NVM code protection disabled)
#pragma config CPD = OFF        // Data NVM Memory Code Protection bit (Data NVM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#define _XTAL_FREQ          32000000
#define SERVO_PERIOD        20000 /* 20 ms servo PWM period */

volatile uint16_t servo_pulse_time = 1500; /* Initial servo pulse time of 1.5 ms */
volatile uint16_t new_pulse_time = 1500; /* Double buffer to avoid glitches */

void __interrupt() isr(void) {
    if (INTCONbits.PEIE) {
        if (PIE4bits.CCP1IE && PIR4bits.CCP1IF) {
            PIR4bits.CCP1IF = 0;
            
            if (CCP1CONbits.CCP1MODE == 0b1000) {            
                CCPR1 += servo_pulse_time;
                CCP1CONbits.CCP1MODE = 0b1001; /* Clear On Match */
            } else {
                CCPR1 += (SERVO_PERIOD - servo_pulse_time);
                CCP1CONbits.CCP1MODE = 0b1000; /* Set On Match */
                servo_pulse_time = new_pulse_time; /* Update time only when output is low */
            }            
        }
    }
}

void init_ports(void) {
    /* All pins in digital mode*/
    ANSELA = 0x00;
    ANSELC = 0x00;
    
    /* Set all pins to output to prevent floating inputs */
    TRISA = 0x00;
    TRISC = 0x00;
}

void init_pps(void) {
    bool state;
    
    /* Preserve global interrupt state and disable interrupts */
    state = INTCONbits.GIE;
    INTCONbits.GIE = 0;
    
    /* Unlock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    /* CCP1 output on RC5 */
    RC5PPS = 0b01100;
    
    /* Lock PPS */
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    /* Restore global interrupt state */
    INTCONbits.GIE = state;
}

void init_timers(void) {
    T1CONbits.TMR1CS = 0b00; /* Timer1 clock source is Fosc/4 */
    T1CONbits.T1CKPS = 0b11; /* Timer1 1:8 prescaler = 1 MHz at Fosc = 32 MHz */
}

void init_ccp(void) {
    CCP1CONbits.CCP1MODE = 0b1000; /* Initial Compare mode is Set On Match */
    CCPTMRSbits.C1TSEL = 0b01; /* CCP1 Compare based on Timer1 */
    
    TMR1 = 0;
    CCPR1 = 10000; /* 10 ms servo startup time */
    T1CONbits.TMR1ON = 1; /* Start Timer1 */
    CCP1CONbits.CCP1EN = 1; /* Enable CCP1 */
}

void init_interrupts(void) {
    PIE4bits.CCP1IE = 1; /* Enable CCP1 interrupt */
    
    INTCONbits.PEIE = 1; /* Enable peripheral interrupts */
}

void init_system(void) {
    init_ports();
    init_pps();
    init_timers();
    init_ccp();
    init_interrupts();
}

void set_servo_pulse_time(uint16_t time) {
    bool status;
    
    status = PIE4bits.CCP1IE;
    PIE4bits.CCP1IE = 0;
    
    new_pulse_time = time;
    
    PIE4bits.CCP1IE = status;
}

void main(void) {
    uint16_t servo_time = 1500;
    
    init_system();
    
    INTCONbits.GIE = 1;
    
    while(1) {
        servo_time = (rand() % 2000) + 500;
        set_servo_pulse_time(servo_time);
        
        __delay_ms(500);
    }
}
