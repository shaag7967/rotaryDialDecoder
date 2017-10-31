/*
 * File:   main.c
 * Author: seven
 *
 * Created on 1. Mai 2017, 11:32
 */

// CONFIG
#pragma config FOSC = INTOSCIO  // Oscillator Selection bits (INTOSC oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF       // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define _XTAL_FREQ 4000000

#include <xc.h>


// Inputs
#define PIN_DIAL            PORTBbits.RB5
#define PIN_NUMBER          PORTBbits.RB4
#define PIN_HOOK_IN         PORTAbits.RA1

// Outputs
#define PIN_NUMBER_VALID    PORTAbits.RA2
#define PIN_HOOK_OUT        PORTAbits.RA3

#define DIAL_ACTIVE       0
#define DELAY_NUMBER_MS   100

typedef unsigned char uint8_t;
typedef unsigned int uint16_t;

uint8_t dialState = 0;
uint8_t dialActive_debounce = 0xFF;

uint8_t number_debounce = 0xAA;
uint8_t numberPinState_current = 0;
uint8_t numberPinState_last = 0;
uint8_t numberEdgeCounter = 0;
uint8_t delay_setNumber = 0;

uint16_t hook_debounce = 0xAA;
uint8_t hookPinState_current = 0;
uint8_t hookPinState_last = 0;


void main(void)
{
    PORTA = 0;
    CMCON = 0x07; // comparators off
    TRISA2 = 0; // number valid
    TRISA3 = 0; // hook
    
    PORTB = 0xFF;
    TRISB = 0xF0;
    
    PIN_NUMBER_VALID = 0;
    
    OPTION_REGbits.nRBPU = 0; // enable pullups

    while(1)
    {
        // rotary
        switch(dialState)
        {
            case 0: // IDLE
                dialActive_debounce <<= 1;
                if(PIN_DIAL)
                    dialActive_debounce |= 0x01;
                
                if(dialActive_debounce == 0x00)
                {
                    dialActive_debounce = 0xFF;
                    numberEdgeCounter = 0;
                    numberPinState_last = numberPinState_current = PIN_NUMBER;
                    dialState = 1;
                }
                break;
            case 1: // 
                if(PIN_DIAL == DIAL_ACTIVE)
                {
                    // debounce signal
                    number_debounce <<= 1;
                    if(PIN_NUMBER)
                        number_debounce |= 0x01;
                    
                    if(number_debounce == 0xFF || number_debounce == 0x00)
                    {
                        // check if signal changed
                        numberPinState_current = number_debounce;
                        if(numberPinState_current != numberPinState_last)
                        {
                            numberEdgeCounter++;
                            numberPinState_last = numberPinState_current;
                        }
                        
                        number_debounce = 0xAA;
                    }
                }
                else
                {
                    PIN_NUMBER_VALID = 0;
                    delay_setNumber = DELAY_NUMBER_MS/2;
                    dialState = 2;
                }
                break;
            case 2:
                delay_setNumber--;
                
                if(delay_setNumber == 0)
                {
                    PORTB = (((numberEdgeCounter >> 1) + 1) % 10) & 0x0F;
                    delay_setNumber = DELAY_NUMBER_MS/2;
                    dialState = 3;
                }
                break;
            case 3:
                delay_setNumber--;
                
                if(delay_setNumber == 0)
                {
                    PIN_NUMBER_VALID = 1;
                    dialState = 0;
                }
                break;
        }
        
        // hook (debounce signal)
        hook_debounce <<= 1;
        if(PIN_HOOK_IN)
            hook_debounce |= 0x01;

        if(hook_debounce == 0xFFFF || hook_debounce == 0x0000)
        {
            // check if signal changed
            hookPinState_current = hook_debounce;
            if(hookPinState_current != hookPinState_last)
            {
                PIN_HOOK_OUT = hookPinState_current;
                hookPinState_last = hookPinState_current;
            }
            
            hook_debounce = 0xAAAA;
        }
        
        __delay_ms(1);
    }
    
    return;
}

