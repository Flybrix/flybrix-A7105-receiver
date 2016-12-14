volatile int state =1;
volatile unsigned int ch[6];
volatile char cur_ch = 0;
static char max_ch = 6;
volatile char cur_ppm = 1500;
volatile char cnt = 12;
volatile int dir = 10;
volatile unsigned char ms = 0;
void setup() {
  
  pinMode(3,OUTPUT);
  PORTB |= (1 << 3);
  
  
  cli();
  TCCR0A = (1 << WGM01)|(0 << WGM00);
  TCCR0B = (0 << WGM02)| (1 <<CS01) | (1 <<CS00);

  OCR0A = 127;

  TIMSK |= (1 << OCIE0A);
 
  sei();
}

void loop() {
  //nothin
  if (ms < 2)
  {  
    PORTB |= (1 << 3);
  
  }
  else if (ms < 6)
  {
    PORTB &= ~(1 << 3);
  }
  else if (ms < 10)
  {
    PORTB |= (1 << 3);
   
  }

  else if (ms < 12)
  {
    PORTB &= ~(1 << 3);
  }
  else if (ms > 12)
  {
    ms = 0;
  }
  

}

ISR(TIM0_COMPA_vect)
{
ms++;
}
/*
if (state) //currently set to 1 go low for 300 us
{
  //  digitalWrite(3, LOW);
  PORTB &= ~(1 << 3);
  state = 0;
    OCR0A = 127;
 
} // current set 0 
else
{
   // digitalWrite(3, HIGH);
  PORTB |= (1 << 3);
  state = 1;
  OCR0A = 127;
 
}
}*/

