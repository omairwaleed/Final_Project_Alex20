#ifndef KIT_IO_H
#define KIT_IO_H

#define relay 2 
#define buzzer 3

#define INT_0 0
#define INT_1 1
#define INT_2 2
#define INT_MODE_LOW 0
#define INT_MODE_ANY 1
#define INT_MODE_FALLING 2
#define INT_MODE_RISING 3

#define Vref_internal 3
#define Vref_AVCC 1
#define Vref_AREF 0

#define PS_2 1
#define PS_4 2
#define PS_8 3
#define PS_16 4
#define PS_32 5
#define PS_64 6
#define PS_128 7

#define Normal 0
#define pwm 1
#define ctc 2
#define fast_pwm 3

#define no_clock 0
#define no_prescaling 1
#define clk_8 2
#define clk_64 3
#define clk_256 4
#define clk_1024 5
#define ext_falling 6
#define ext_rising 7

#define enable 1
#define disable 0

#define spi_master  1
#define spi_slave  0
#define spi_4   0
#define spi_16   1
#define spi_64  2
#define spi_128  3
#define spi_2  4
#define spi_8  5  
#define spi_32  6
//#define spi_64  7
#define MOSI 5
#define MISO 6
#define SS 4
#define SCK 7
// keypad
#define Key_1 0
#define Key_2 1
#define Key_3 2
#define Key_a 4
#define Key_b 5
#define Key_c 6
#define Key_d 7

#define _4BITS    4
#define _8BITS    8



void relay_init();
void ledinit();
void led0_on();
void led1_on();
void led2_on();
void led0_OFF();
void led1_OFF();
void led2_OFF();
void buzzer_init();
void buzzer_OFF();
void buzzer_on();
void relay_off();
void relay_on();
void btns_init();
int ispressedbtn2();
int ispressedbtn0();
int ispressedbtn1();
void delay();
void toggleled0();
void ifclicked();
void intenable(int int_name, int int_mode);
void int_disable(int int_name);
void ADC_selectVref(int Vref);
void ADC_selectCH(int channel);
void ADC_selectPrescaler(int Prescaler);
void ADC_Enable();
void ADC_Disnable();
void ADC_Start();
void ADC_Init(int chanel, int Vref, int Prescaler);
int ADC_Read();
int ADC_Read_l();
void Timer0_init(int mode, int clk);
void Timer0_OVIE(int state);
void timer_ocie(int state);
void timer0_setMaxPoint(unsigned char val);
void application_on_adc();
void UART_init();
void UART_setBaudRate(int bauderate );
void UART_send(char data);
char UART_recieve();
void UART_send_string(char*str);
void UART_send_num(int num);
void spi_send(char c);
char spi_receive();
void spi_initialize(int type,int clck);

void LCD_cmd(char);

void LCD_init(int mode);  // initiate driver
void LCD_clear();
void LCD_write(char); // location???

void LCD_write_str(char*);
void LCD_write_num(unsigned int);

void LCD_goto_line1();
void LCD_goto_line2();

void LCD_goto_xy(int , int);


void LCD_cmd_4bits(char);
void LCD_write_4bits(char );
void LCD_clear_4bits();


void LCD_write_str_4bits(char*);
void LCD_write_num_4bits(unsigned int);

void LCD_goto_line1_4bits();
void LCD_goto_line2_4bits();

void LCD_goto_xy_4bits(int , int);
#endif