#include <avr/io.h>
#include <avr/interrupt.h>
#include "lcd.h"

volatile uint16_t softpwmval = 0;
const uint16_t softpwmvalmax = 1000;

volatile uint16_t current_time = 0;
volatile uint16_t best_time = 999;
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
  if (level >= softpwmvalmax){
    level = softpwmvalmax;
  }
  softpwmval = level;
}

void TimerInit(){
  OCR1A = 62500; // 1Hz clock
  TCCR1B |= (1 << WGM12);  // CTC mode
  TCCR1B |= (1 << CS12); // TCLK/256
}

void TimerStart(){
  current_time = 0;
  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A); // Enable Timer1 interrupt
}

void TimerStop(){
  TIMSK1 &= ~(1 << OCIE1A); // Disable Timer1 interrupt
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

void lcd_putint(uint16_t q){
  uint16_t tmp;

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

static const PROGMEM unsigned char graphical_chars[] =
{
  0x02, 0x04, 0x0E, 0x01, 0x0F, 0x11, 0x0F, 0x00, // á
  0x02, 0x04, 0x0E, 0x11, 0x1F, 0x10, 0x0E, 0x00, // é
  0x0A, 0x0A, 0x00, 0x0E, 0x11, 0x11, 0x0E, 0x00, // ő
  0x02, 0x04, 0x00, 0x0E, 0x11, 0x11, 0x0E, 0x00, // ó
  0x04, 0x04, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x00, // Ú
  0x12, 0x14, 0x18, 0x10, 0x18, 0x15, 0x13, 0x17, // transistor
  0x0E, 0x0E, 0x04, 0x0E, 0x15, 0x04, 0x0A, 0x0A, // player
  0x00, 0x0A, 0x00, 0x1F, 0x11, 0x0E, 0x00, 0x00 // smiley
};

typedef enum{aa=0, ee=1, oeoe=2, oo=3, UU=4, transistor=5, player=6, smiley = 7} spec_char;

void lcd_put_spec(uint8_t x, uint8_t y, spec_char chr){
  lcd_gotoxy(x,y);
  lcd_putc(chr);
}

void lcd_load_spec(void){
  lcd_command(_BV(LCD_CGRAM));
  for(uint8_t i=0; i<64; i++)
  {
    lcd_data(pgm_read_byte_near(&graphical_chars[i]));
  }
}

int main(void){
  
  TaserInit();
  TimerInit();
  GameInit();
  lcd_init(LCD_DISP_ON);
  lcd_load_spec();

  lcd_clrscr();
  lcd_gotoxy(0,0);
  lcd_puts(" = Razos jatek =");
  lcd_put_spec(1, 0, transistor);
  lcd_put_spec(4, 0, aa);
  lcd_put_spec(6, 0, oo);
  lcd_put_spec(10, 0, aa);
  lcd_put_spec(12, 0, ee);
  lcd_put_spec(15, 0, player);

  lcd_gotoxy(0,1);
  lcd_puts("Menj a rajthoz! ");
  while(!StartTouched()){};  

  sei();  // Enable interrupts

  while(1){
    switch (state){
    case START:
      TaserSetLevel(0);

      lcd_clrscr();
      lcd_gotoxy(0,0);
      lcd_puts("Indulhat a jatek");
      lcd_put_spec(12, 0, aa);
      lcd_put_spec(14, 0, ee);
      lcd_gotoxy(0,1);
      lcd_puts("Rekordido:    s ");
      lcd_put_spec(8, 1, oeoe);
      lcd_gotoxy(11,1);
      lcd_putint(best_time);

      while(StartTouched()){};

      lcd_clrscr();
      lcd_gotoxy(0,0);
      lcd_puts("Rekordido:    s ");
      lcd_put_spec(8, 0, oeoe);
      lcd_gotoxy(11,0);
      lcd_putint(best_time);
      lcd_gotoxy(0,1);
      lcd_puts("Jatekido:    0s ");
      lcd_put_spec(1, 1, aa);
      lcd_put_spec(3, 1, ee);
      lcd_put_spec(7, 1, oeoe);
      TaserSetLevel(20);
      TimerStart();
      state = PLAYING;
      break;

    case PLAYING:
      if(update_display){
        update_display = 0;
        // increase PWM value gradually
        TaserSetLevel(softpwmval + 5);
        lcd_gotoxy(11,1);
        lcd_putint(current_time);
      }
      if(FinishTouched()){
        state = FINISH;
      }else if(StartTouched()){
        state = START;
      }
      break;

    case FINISH:
      TaserSetLevel(0);
      TimerStop();

      lcd_clrscr();
      lcd_gotoxy(0,0);
      lcd_puts(" Celba ertel!   ");
      lcd_put_spec(2, 0, ee);
      lcd_put_spec(7, 0, ee);
      lcd_put_spec(10, 0, ee);
      lcd_put_spec(14, 0, smiley);

      lcd_gotoxy(0,1);
      if(best_time>current_time){
        best_time = current_time;
        lcd_puts("Uj rekord!    s ");
        lcd_put_spec(0, 1, UU);
        lcd_gotoxy(11,1);
        lcd_putint(best_time); 
      }else{
        lcd_puts("Grat! Idod:    s");
        lcd_put_spec(8, 1, oeoe);
        lcd_gotoxy(12,1);
        lcd_putint(current_time); 
      }

      while(!StartTouched()){};
      state = START;
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
  if(current_time>999){
    current_time = 999;
  }
  update_display = 1;
}
