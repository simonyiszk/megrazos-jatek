/*
 * megrazos_jatek.c
 *
 * Created: 2017.04.11. 19:58:48
 *  Author: SEM
 */ 


#include <avr/io.h>
#include "lcd.h"
#include <avr/interrupt.h>
#include <util/delay.h>

#define startpin()  !(PINB&(1<<PB1))
#define endpin()  !(PINB&(1<<PB2))
#define softpwmvalmax 1000

volatile uint16_t softpwmval;

volatile uint8_t current_time;

volatile uint16_t best_time = 254;

inline void set_best_time(uint8_t);
inline void set_current_time();
void lcd_putint(uint8_t);

volatile unsigned char time_changed;

enum statevar {
	start,
	playing,
	finish} state;

uint8_t enteringstate;

void set_best_time(uint8_t s){
	lcd_gotoxy(11,0);
	lcd_putint(s);
}

void set_current_time(){
	lcd_gotoxy(11,1);
	lcd_putint(current_time);
}

void lcd_putint(uint8_t q){
	uint8_t tmp;
	
	tmp = q/100;
	if(tmp){
		lcd_putc('0'+tmp);
		tmp = (q%100)/10;
		lcd_putc('0'+tmp);
	}
	else{
		lcd_putc(' ');	
		tmp = (q%100)/10;
		if(tmp)
			lcd_putc('0'+tmp);
		else
			lcd_putc(' ');
	}

	tmp = q%10;
	lcd_putc('0'+tmp);
}

void gotostate(uint8_t newstate)
{
	if (state != newstate){ 
		state = newstate;
		enteringstate = 1;
	}
} 

int main(void)
{
	
	DDRD |= (1<<PD4);
	PORTD &=~ (1<<PD4);
	
	DDRB |= (1<<PB3); // tekercs tranzisztor PWM laba
	TCCR2 |= (1<<CS21);
	//TCCR2 |= (1<<WGM20) | (1<<WGM21) | (1<<COM21);
	TIMSK |= (1<<TOIE2);
	
	softpwmval=0;
	
	//OCR2 = 50;
	
	///////////////////////////INIT VARS
	current_time = 0;
	time_changed = 0;
	enteringstate = 1;
	
	///////////////////////////LCD INIT
	
	lcd_init(LCD_DISP_ON);
	
	////////////////////////TIMER INIT
	OCR1A=46875;
	TCCR1A = 1 << WGM12;
	TIMSK |= 1 << OCIE1A;
	TCCR1B = 1 << CS12;	
	
	sei();
	
    while(1)
    {
		//main Q&D SM
		
		switch (state){
			
			case start:
				if (enteringstate){
					softpwmval=0;
					lcd_clrscr();
					lcd_gotoxy(0,0);
					lcd_puts("Ready to play!");
					lcd_gotoxy(0,1);
					lcd_puts("Best time:    s");
					lcd_gotoxy(11,1);
					lcd_putint(best_time);
					enteringstate = 0;
				}
				if (!startpin()) gotostate(playing);
			break;
			
			case playing:
				if (enteringstate){
					lcd_clrscr();
					lcd_puts("Best time:    s");
					lcd_gotoxy(11,0);
					lcd_putint(best_time);
					lcd_gotoxy(0,1);  
					lcd_puts("Current:      s");
					softpwmval=20;
					current_time=0;
					enteringstate = 0;
				}
						if (time_changed){
							softpwmval += 5;
							if (softpwmval >= softpwmvalmax) softpwmval = softpwmvalmax;
							time_changed = 0;
							cli();
							lcd_gotoxy(11,1);
							lcd_putint(current_time);
							sei();
							if (startpin()) gotostate(start);
							if (endpin()) gotostate(finish);
						}
			break;
			
			case finish:
			if (enteringstate){
					softpwmval=0;
					if (current_time < best_time) best_time = current_time;
					lcd_clrscr();
					lcd_gotoxy(0,0);
					lcd_puts("Grat!       s");
					lcd_gotoxy(11,0);
					lcd_putint(current_time);
					lcd_gotoxy(0,1);
					lcd_puts("Best time:    s");
					lcd_gotoxy(11,1);
					lcd_putint(best_time);
					enteringstate = 0;
				}
			if (startpin()) gotostate(start);
			break;
		}
				
    }
}

volatile uint16_t softpwmcntr;
ISR (TIMER2_OVF_vect){	// gyors INTerrupt
	softpwmcntr++;
	if (softpwmcntr > softpwmval) 
		PORTB &=~ (1<<PB3);
	else PORTB |= (1<<PB3);
	if (softpwmcntr > 1024) softpwmcntr=0; // 10bit soft pwm 
}

ISR(TIMER1_COMPA_vect){
	current_time++;
	time_changed=1;
}
