/*
 * megrazos_jatek.c
 *
 * Created: 2017.04.11. 19:58:48
 *  Author: SEM
 */ 


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

enum states {Before_After}

int main(void)
{
	DDRB |= (1<<PB3); // tekercs tranzisztor PWM laba
	current_time = 0;
	
	OCR1A=46875;
	TCCR1A = 1 << WGM12;
	TIMSK = 1 << OCIE1A;
	TCCR1B = 1 << CS12;	
	
	sei();
	
    while(1)
    {
		/*
		PORTB |= (1<<PB3);
        _delay_ms(50);
		PORTB &=~ (1<<PB3);
		_delay_ms(500);
		*/
		switch(state)
		case
		
		
		
		default:
    }
}
