#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>
#include"Kit2.h"
#include <avr/interrupt.h>

#include <stdlib.h>

#define RW           2
#define RS			 1
#define EN			 3
#define ctrl		 PORTB
#define LCD_data	 PORTA

#define ctrl_dir     DDRB
#define LCD_data_dir DDRA

void ledinit() {
    //DATA DIRECTION(INPUT OR OUTPUT)
    DDRC |= (1 << 2) | (1 << 7);
    DDRD |= (1 << 3);
}

void led0_on() {
    //DEFINE DATA (HIGH ,LOW)
    PORTC |= (1 << 2);
}
void UART_init(int bauderate ){
     UCSRB |= (1<<TXEN)|(1<<RXEN);
   //  UCSRB |= (1<<RXCIE);
    UART_setBaudRate( bauderate );
}
void UART_send(char data){
    while(!(UCSRA&(1<<UDRE)));//UDR IS EMPTY TO TAKE DATA
    UDR=data;
}
void UART_setBaudRate(int bauderate ){
    int x=(((F_CPU/16.0)/bauderate)-1);
    UBRRL=(char)x;
    UBRRH=(x>>8);
}
char UART_recieve(){
    while(!(UCSRA&(1<<RXC)));//receive shift register is complete and push to UDR
    return UDR;
}
void UART_send_string(char*str){
    for(int i=0;str[i]!='\0';i++)
        UART_send(str[i]);
}

void UART_send_num(int num){
    char buff [11];
    itoa(num,buff,10);
    UART_send_string(buff);
}
void spi_initialize(int type,int clck){
    
  
    switch (type){
        case spi_master:
             SPCR |= (1<<MSTR);
             SPCR |= (clck>3)?(clck & 0xFD):clck;
             SPCR |= (clck>3)?(1<<SPI2X):0;
            // SPCR |= (1<<0)|(1<<1);
             DDRB |= (1<<MOSI)|(1<<SCK)|(1<<SS);
            break;
        case spi_slave:
             SPCR &= ~(1<<MSTR);
             DDRB |= (1 << MISO);
            break;
    }
    SPCR |= (1<<SPE);
}

void spi_send(char c){
    SPDR = c;
   
    while(!(SPSR &(1<<SPIF)));
    
}

char spi_receive(){
    while(!(SPSR&(1<<SPIF)));
    return SPDR;
}


//driver for microcontroller but not used in this project
void relay_init() {
    DDRA |= (1 << relay);
}

void relay_on() {
    PORTA |= (1 << relay);
}

void led1_on() {
    //DEFINE DATA (HIGH ,LOW)
    PORTC |= (1 << 7);
}

void led2_on() {
    //DEFINE DATA (HIGH ,LOW)
    PORTD |= (1 << 3);
}

void buzzer_init() {
    DDRA |= (1 << buzzer);
}

void buzzer_on() {
    PORTA |= (1 << buzzer);
}

void led0_OFF() {
    PORTC &= ~(1 << 2);
}

void led1_OFF() {
    PORTC &= ~(1 << 7);
}

void led2_OFF() {
    PORTD &= ~(1 << 3);
}

void buzzer_OFF() {
    PORTA &= ~(1 << buzzer);
}

void relay_off() {
    PORTA &= ~(1 << relay);
}

void btns_init() {
    DDRD &= ~((1 << 2) | (1 << 6));
    DDRB &= ~((1 << 0));
}

int ispressedbtn0() {
    if (PINB & (1 << 0))
        return 1;
    else
        return 0;
}

int ispressedbtn1() {
    if (PIND & (1 << 6))
        return 1;
    else
        return 0;
}

int ispressedbtn2() {
    if (PIND & (1 << 2))
        return 1;
    else
        return 0;
}

void toggleled0() {
    PORTC ^= (1 << 2);
}

void delay() {
    led0_on();
    led1_on();
    led2_on();
    buzzer_on();
    _delay_ms(1000);
    led0_OFF();
    led1_OFF();
    led2_OFF();
    buzzer_OFF();
    _delay_ms(1000);
}

void f2() {
    if (ispressedbtn0()) {
        led0_on();
    } else
        led0_OFF();
    if (ispressedbtn1()) {
        led1_on();
    } else
        led1_OFF();
    if (ispressedbtn2()) {
        led2_on();
    } else
        led2_OFF();

    if (ispressedbtn0() && ispressedbtn1()) {
        buzzer_on();
    } else
        buzzer_OFF();

}

void ifclicked() {
    if (ispressedbtn0()) {
        toggleled0();
        _delay_ms(500);
    }
}

void intenable(int int_name, int int_mode) {
    if (int_name == INT_0) {
        GICR &= ~(1 << INT0);
        switch (int_mode) {
            case INT_MODE_LOW:
                MCUCR &= ~((1 << ISC00) | (1 << ISC01));
                break;
            case INT_MODE_ANY:
                MCUCR |= (1 << ISC00);
                MCUCR &= ~(1 << ISC01);
                break;
            case INT_MODE_FALLING:
                MCUCR |= (1 << ISC01);
                MCUCR &= ~(1 << ISC00);
                break;
            case INT_MODE_RISING:
                MCUCR |= (1 << ISC00) | (1 << ISC01);
                break;
        }
        GICR |= (1 << INT0);
    } else if (int_name == INT_1) {
        GICR &= ~(1 << INT1);
        switch (int_mode) {
            case INT_MODE_LOW:
                MCUCR &= ~(1 << ISC10) | (1 << ISC11);
                break;
            case INT_MODE_ANY:
                MCUCR |= (1 << ISC10);
                MCUCR &= ~(1 << ISC11);
                break;
            case INT_MODE_FALLING:
                MCUCR |= (1 << ISC11);
                MCUCR &= ~(1 << ISC10);
                break;
            case INT_MODE_RISING:
                MCUCR |= (1 << ISC10) | (1 << ISC11);
                break;
        }
        GICR |= (1 << INT1);
    } else if (int_name == INT_2) {
        GICR &= ~(1 << INT2);
        switch (int_mode) {

            case INT_MODE_FALLING:
                MCUCSR &= ~(1 << ISC2);

                break;
            case INT_MODE_RISING:
                MCUCSR |= (1 << ISC2);
                break;
        }
        GICR |= (1 << INT2);
    } else {
        //# warning "DUMMY";
    }
    //you must call sei(); for gloabl enable of interrupt.
}

void int_disable(int int_name) {
    switch (int_name) {
        case INT_0:
            GICR &= ~(1 << INT0);
            break;
        case INT_1:
            GICR &= ~(1 << INT1);
            break;
        case INT_2:
            GICR &= ~(1 << INT2);
            break;
    }

}

void ADC_selectVref(int Vref) {
    ADMUX &= ~((1 << REFS1) | (1 << REFS0));
    switch (Vref) {
        case Vref_internal:
            ADMUX |= ((1 << REFS1) | (1 << REFS0));
            break;

        case Vref_AVCC:
            ADMUX |= (1 << REFS0);
            break;

        case Vref_AREF:
            ADMUX &= ~((1 << REFS1) | (1 << REFS0));
            break;
    }

}

void ADC_selectCH(int channel) {
    //ADMUX &= ~((1 << MUX0) | (1 << MUX1) | (1 << MUX2) | (1 << MUX3) | (1 << MUX4));
    ADMUX &= 0xE0;
    ADMUX |= channel;
}

void ADC_selectPrescaler(int Prescaler) {
    ADCSRA &= 0xF8;
    ADCSRA |= Prescaler;
}

void ADC_Enable() {
    ADCSRA |= (1 << ADEN);

}

void ADC_Disnable() {
    ADCSRA &= ~(1 << ADEN);
}

void ADC_Start() {
    ADCSRA |= (1 << ADSC);
}

void ADC_Init(int chanel, int Vref, int Prescaler) {
    ADC_selectPrescaler(Prescaler);
    ADC_selectCH(chanel);
    ADC_selectVref(Vref);
    ADC_Enable();
}

int ADC_Read() {
    int data = ADCL;
    data |= (ADCH << 8);
    return data;
}

int ADC_Read_l() {
    int data = ADCL;
    data |= (ADCH << 8);
    return (data >> 6);
}

void Timer0_init(int mode, int clk) {
    switch (mode) {
        case Normal:
 TCCR0 &= ~((1<<WGM00)|(1<<WGM01));
            break;
        case pwm:
TCCR0 &= ~(1<<WGM01);
TCCR0 |= (1<<WGM00);
            break;
        case ctc:
TCCR0 &= ~(1<<WGM00);
TCCR0 |= (1<<WGM01);
            break;
        case fast_pwm:
 TCCR0 |= ((1<<WGM00)|(1<<WGM01));
            break;

    }
    TCCR0 &= ~((1 << CS00) | (1 << CS01) | (1 << CS02));
    TCCR0 |= clk;

}

void Timer0_OVIE(int state){//to enable interrupt 
if (state)
    TIMSK |=(1<<TOIE0);
else
TIMSK &=~(1<<TOIE0);
    
}

void application_on_adc(){
     ADC_Init(1, Vref_AVCC, PS_128);
    LCD_init(_4BITS);
//    LCD_write_num_4bits(55);
    while (1) {
        _delay_ms(200);
        ADC_Start();
        while (!(ADCSRA & (1 << ADIF)));
        LCD_clear_4bits();
        LCD_write_num_4bits(ADC_Read()*4.8875855327468230694037145650049);
    }
}
void timer0_setMaxPoint(unsigned char val){//to enable interrupt on ctc mmode
    OCR0=val;
}
void timer_ocie(int state){
    if (state)
        TIMSK |= (1<<OCIE0);
    else
          TIMSK &= ~(1<<OCIE0);
}

void app_on_uart(){
       ADC_Init(0, Vref_AREF, PS_128);
    
    UART_init(9600);
    //sei();
    while(1){
     _delay_ms(500);
      ADC_Start();
        while (!(ADCSRA & (1 << ADIF)));
        UART_send_num(ADC_Read()*4.8875855327468230694037145650049);
        UART_send('\r');
    }
}
void ADC_WAIT(){
    while(!(ADCSRA & (1<<ADIF)));
}

void LCD_cmd(char cmd){
	ctrl &= ~(1<<RS); // access to Command Register
	LCD_data = cmd;
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_ms(100);
	ctrl &= ~(1<<EN); // Falling Edge
	
}

void LCD_cmd_4bits(char cmd){
	ctrl &= ~(1<<RS); // access to Command Register
	
	LCD_data &= 0x0F;
	LCD_data |= (0xF0&cmd);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_us(200);
	LCD_data &= 0x0F;
	LCD_data |= (cmd<<4);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_ms(2);
	
}

void LCD_write_4bits(char data){
	ctrl |= (1<<RS); // access to Data Register
	
	LCD_data &= 0x0F;
	LCD_data |= (0xF0&data);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_us(200);
	LCD_data &= 0x0F;
	LCD_data |= (data<<4);
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_us(1);
	ctrl &= ~(1<<EN); // Falling Edge
	
	_delay_ms(2);
	
}

void LCD_init(int mode){
	// Data Direction
	
	ctrl_dir |= (1<<RS)|(1<<EN)|(1<<RW);
	ctrl &= ~(1<<RW);
	
	
	
	
	if(mode == _4BITS){
		LCD_data_dir |= 0xF0;  // PORTx as OUTPUT
		LCD_cmd(0x20);                //Data Mode  4-bits
		_delay_ms(1);
		LCD_cmd_4bits(0x01);          // make clear LCD
		LCD_cmd_4bits(0x0C);          // display on, cursor off
		_delay_ms(100);
	}
	
	else{
		LCD_data_dir = 0xFF;  // PORTx as OUTPUT
		// initiate LCD for 8bit mode.// Data Mode  8-bits
		LCD_cmd(0x38);
		_delay_ms(1);
		LCD_cmd(0x01);          // make clear LCD
		LCD_cmd(0x0C);          // display on, cursor off
		_delay_ms(100);
	}
}

void LCD_write(char data){
	
	LCD_data = data;
	ctrl |= (1<<RS);
	
	
	ctrl |= (1<<EN);  // Rising Edge
	_delay_ms(1);
	ctrl &= ~(1<<EN); // Falling Edge
}

void LCD_clear(){
	_delay_ms(1);
	LCD_cmd(0x01);          // make clear LCD
	_delay_ms(1);
}
void LCD_clear_4bits(){
	_delay_ms(1);
	LCD_cmd_4bits(0x01);          // make clear LCD
	_delay_ms(1);
}

void LCD_write_str(char* str){
	
	for(int i=0; str[i]!= '\0';i++){
		LCD_write(str[i]);
	}
}
void LCD_write_str_4bits(char* str){
	
	for(int i=0; str[i]!= '\0';i++){
		LCD_write_4bits(str[i]);
	}
}

void LCD_write_num(unsigned int num){
	char strNumber[8];
	
	itoa(num, strNumber,10);
	
	LCD_write_str(strNumber);
	
}

void LCD_write_num_4bits(unsigned int num){
	char strNumber[8];
	
	itoa(num, strNumber,10);
	
	LCD_write_str_4bits(strNumber);
	
}


void LCD_goto_line1(){
	LCD_cmd(0X80);
}

void LCD_goto_line2(){
	LCD_cmd(0XC0);
}

void LCD_goto_line1_4bits(){
	LCD_cmd_4bits(0X80);
}

void LCD_goto_line2_4bits(){
	LCD_cmd_4bits(0XC0);
}

void LCD_goto_xy(int row , int col){  // col 0 ~ 15      0 ~ F
	if(row == 0 && col < 16){  // LINE 1
		LCD_cmd(0X80|col);
	}
	else if (row == 1 && col < 16) // LINE 2
	{
		LCD_cmd(0XC0|col);
	}
	else{
		//LCD_write_str("out of dimension");
	}
}

void LCD_goto_xy_4bits(int row , int col){  // col 0 ~ 15      0 ~ F
	if(row == 0 && col < 16){  // LINE 1
		LCD_cmd_4bits(0X80|col);
	}
	else if (row == 1 && col < 16) // LINE 2
	{
		LCD_cmd_4bits(0XC0|col);
	}
	else{
		//LCD_write_str_4bits("out of dimension");
	}
}
