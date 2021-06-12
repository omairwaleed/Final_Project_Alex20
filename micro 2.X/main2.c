/*
 * File:   main2.c
 * Author: omair waleed
 *
 * Created on June 7, 2021, 2:39 PM
 */


#define F_CPU 16000000UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/iom32a.h>
#include "Kit2.h"

int main(void) {

    char data;
    UART_init(9600);
    ledinit();
   // _delay_ms(1000);
    spi_initialize(spi_slave, spi_128);

    while (1) {
        data = spi_receive();
        if (data == 'a') {
            led0_on();
        } else if (data == 'b') {
            led0_OFF();
        } else if (data == 'c') {
            led1_on();
        } else if (data == 'd') {
            led1_OFF();
        }


    }

    return 0;
}
