#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"

volatile uint16_t softpwmval = 0;
const uint16_t softpwmvalmax = 1000;

volatile uint8_t current_time = 0;
volatile uint16_t best_time = 254;
volatile uint8_t update_display = 0;

enum gamestate {
  START,
  PLAYING,
  FINISH
} state;

void TaserInit(){
  DDRB |= (1<<PB3); // PWM output pin
  
  TCCR2B |= (1<<CS21); // TCLK/8 (from prescaler, eredetileg 12MHz, most 16MHz)
  TIMSK2 |= (1<<TOIE2); // Enable Timer2 interrupt
  // TODO: implement HW PWM instead
}

void TaserSetLevel(uint16_t level){
  if (level >= softpwmvalmax) level = softpwmvalmax;
  softpwmval = level;
}

void TimerInit(){
  OCR1A = 62500; // 1Hz clock
  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS12); // TCLK/256
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
}

void GameInit(){
  DDRB &= ~(1 << PINB1);  // Set START pin as input
  DDRB &= ~(1 << PINB2);  // Set FINISH pin as input
  state = START;
}

static inline uint8_t StartTouched(void){
  return (((PINB & _BV(PINB1))==0) ? 1 : 0);
}

static inline uint8_t FinishTouched(void){
  return (((PINB & _BV(PINB2))==0) ? 1 : 0);
}

void lcd_putint(uint8_t q){
  uint8_t tmp;

  tmp = q/100;
  if(tmp){
    lcd_putc('0'+tmp);
    tmp = (q%100)/10;
    lcd_putc('0'+tmp);
  }else{
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

int main(void){
  
  TaserInit();
  TimerInit();
  GameInit();
  lcd_init(LCD_DISP_ON);

  sei();  // Enable interrupts

  while(1){
    switch (state){
    case START:
      TaserSetLevel(0);
      lcd_clrscr();
      lcd_gotoxy(0,0);
      lcd_puts("Ready to play!");
      lcd_gotoxy(0,1);
      lcd_puts("Best time:    s");
      lcd_gotoxy(11,1);
      lcd_putint(best_time);

      while(StartTouched()){};
      
      lcd_clrscr();
      lcd_puts("Best time:    s");
      lcd_gotoxy(11,0);
      lcd_putint(best_time);
      lcd_gotoxy(0,1);  
      lcd_puts("Current:      s");
      TaserSetLevel(20);
      current_time = 0;
      state = PLAYING;
      break;

    case PLAYING:
      if(update_display){
        update_display = 0;
        // increase PWM value gradually
        TaserSetLevel(softpwmval + 5);
        lcd_gotoxy(11,1);
        lcd_putint(current_time);
        if(FinishTouched()){
          state = FINISH;
        }else if(StartTouched()){
          state = START;
        }
      }
      break;

    case FINISH:
      TaserSetLevel(0);
      // TODO save new record if any
      lcd_clrscr();
      lcd_gotoxy(0,0);
      lcd_puts("Grat!       s");
      lcd_gotoxy(11,0);
      lcd_putint(current_time);
      lcd_gotoxy(0,1);
      lcd_puts("Best time:    s");
      lcd_gotoxy(11,1);
      lcd_putint(best_time);
      if(StartTouched()){
        state = START;
      }
      break;
      
    default:
      break;
    }
  }
}

ISR(TIMER2_OVF_vect){
  static volatile uint16_t softpwmcntr;
  softpwmcntr++;
  if (softpwmcntr > softpwmval){
    PORTB &=~ _BV(PORTB3);
  }else{
    PORTB |= _BV(PORTB3);
  }
  if (softpwmcntr > 1024) softpwmcntr=0; // 10bit soft pwm 
}

ISR(TIMER1_COMPA_vect){
  current_time++;
  update_display = 1;
}