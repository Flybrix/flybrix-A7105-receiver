volatile int state =1;
volatile unsigned int ch[6];
volatile char cur_ch = 0;
static char max_ch = 6;
volatile char cur_ppm = 1500;
volatile char cnt = 12;
volatile int dir = 10;

void setup() {
  
  pinMode(3,OUTPUT);
  PORTB |= (1 << 3);
  int i;
  for (i =0; i < 12; i++)
  {
   ch[i] = 1500;
  }
  
  cli();
  //TCCR1A |= (1 << WGM01)|(0 << WGM00);
  //TCCR1B |= (0 << WGM02)| (1 <<CS00);
  TCCR1 = 128 | 5;
  
  //TCCR1 = 128 | 5;
  

    OCR1C = 148; //low for 300 us

//  TIMSK |= (1 << OCIE1A);
    TIMSK |= (1 << TOIE1);
 
  sei();
}

void loop() {
  //nothin
  

}

//ISR(TIM1_COMPA_vect)
ISR(TIM1_OVF_vect)

{

if (state) //currently set to 1 go low for 300 us
{
  //  digitalWrite(3, LOW);
  PORTB &= ~(1 << 3);
  state = 0;
  TCCR1 = 128 | 5;
  OCR1C = 148; //low for 300 us

//  OCR1C = 150; //low for 300 us

} // current set 0 
else
{
   // digitalWrite(3, HIGH);
  PORTB |= (1 << 3);
  state = 1;

 
 if (cur_ch == max_ch)
 { 
    ch[0] += dir;
  if (ch[0] > 1900)
  {
   dir = -10;
  }
  if (ch[0] < 1100)
  {
   dir = 10;
  }

  //go high for 24ms - (ch0 + ch1 + ch2 + ch3 + ch4 + ch5)
  TCCR1 = 128 | 13;
  OCR1C = 32;
  cur_ch = 0;
 }
 else  
 {//go high for (ch[cur_ch] - 300) microseconds;
  int us = (ch[cur_ch] - 300);
    TCCR1 = 128 | 7;
    OCR1C = (char) (us >> 3);
   cur_ch++;
 }
 
}

}

