/*
 * File:   main1.c
 * Author: omair waleed
 *
 * Created on June 7, 2021, 2:31 PM
 */


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/iom32a.h>
#include "Kit.h"



int main(void) {
    char x;
    UART_init(9600);
    spi_initialize(spi_master, spi_128);
 
    
    while (1) {
        x=UART_recieve();
        spi_send(x);
        
       

    }
    return 0;
}
