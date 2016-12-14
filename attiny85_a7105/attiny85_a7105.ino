#include "common.h"
extern "C" {
#include "A7105_spi.h"
#include "A7105.h"
}




volatile int state =1;
volatile unsigned int ch[6];
volatile unsigned int n_ch[6];
volatile char cur_ch = 0;
static char max_ch = 6;
volatile char cur_ppm = 1500;
volatile char cnt = 12;
volatile int dir = 10;
volatile unsigned char ms = 0;




#define AUX1_FLAG   0x04 
#define AUX2_FLAG   0x08 

static const uint8_t allowed_ch[] = {0x14, 0x1E, 0x28, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x6E, 0x78, 0x82};
static uint8_t packet[16], channel, counter;
static uint8_t txid[4];
static unsigned long timeout_timer;
void init_a7105(void);
bool hubsan_check_integrity(void);
void update_crc(void);

void bind(void);

#define A7105_WriteRegister SPI_reg_write
#define A7105_ReadRegister SPI_reg_read
#define A7105_Strobe SPI_single_write
unsigned char transmitting = 0;
unsigned char wait_for_read_rx = 0;

void setup()
{
  pinMode(3,OUTPUT);
  PORTB |= (1 << 3);
  int i;
  for (i =0; i < 12; i++)
  {
   ch[i] = 1500;
  }
  
  cli();
  
 TCCR1 = 128 | 5;
 OCR1C = 148; //low for 300 us
 

  TCCR0A = (1 << WGM01)|(0 << WGM00);
  TCCR0B = (0 << WGM02)| (1 <<CS01) | (1 <<CS00);
  OCR0A = 127;
 TIMSK |= (1 << OCIE0A);


 
 sei();

 
 SPI_init();
 ms = 0;
  while(ms < 10) {
    }
  
  ms = 0;
  
  init_a7105();
  bind();
 TIMSK |= (1 << TOIE1);
  A7105_Strobe(A7105_RX);
}

ISR(TIM0_COMPA_vect)
{
ms++;
return;
}

ISR(TIM1_OVF_vect)

{

/*if (wait_for_read_rx == 1)
{
   TCCR1 = 128 | 13;
  OCR1C = 32;
  return; //wait another ms
}*/
if (cur_ch == 0 && transmitting == 0)
{
  transmitting = 1;
}
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
 /*   ch[0] += dir;
  if (ch[0] > 1900)
  {
   dir = -10;
  }
  if (ch[0] < 1100)
  {
   dir = 10;
  }*/

  //go high for 24ms - (ch0 + ch1 + ch2 + ch3 + ch4 + ch5)
  TCCR1 = 128 | 13;
  OCR1C = 32;//was 32
  cur_ch = 0;
  transmitting = 0;
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





void update_crc(void)
{
    int sum = 0;
    for(int i = 0; i < 15; i++)
        sum += packet[i];
    packet[15] = (256 - (sum % 256)) & 0xff;
}

bool hubsan_check_integrity(void) 
{
    int sum = 0;
    for(int i = 0; i < 15; i++)
        sum += packet[i];
    return packet[15] == ((256 - (sum % 256)) & 0xff);
}

void hubsan_build_bind_packet(uint8_t bindstate)
{
    packet[0] = bindstate;
    packet[1] = (bindstate!=0x0a)? channel : counter;
    packet[6] = 0x08;
    packet[7] = 0xe4;
    packet[8] = 0xea;  
    packet[9] = 0x9e;
    packet[10] = 0x50;
    update_crc();
}

void init_a7105(void)
{
    A7105_reset();
    A7105_ID_write(0x55201041); 
    A7105_WriteRegister(A7105_01_MODE_CONTROL, 0x63);
    A7105_WriteRegister(A7105_03_FIFOI, 0x0f);
    A7105_WriteRegister(A7105_0D_CLOCK, 0x05);
    A7105_WriteRegister(A7105_0E_DATA_RATE, 0x04);
    A7105_WriteRegister(A7105_15_TX_II, 0x2b);
    A7105_WriteRegister(A7105_18_RX, 0x62);
    A7105_WriteRegister(A7105_19_RX_GAIN_I, 0x80);
    A7105_WriteRegister(A7105_1C_RX_GAIN_IV, 0x0A);
    A7105_WriteRegister(A7105_1F_CODE_I, 0x07);
    A7105_WriteRegister(A7105_20_CODE_II, 0x17);
    A7105_WriteRegister(A7105_29_RX_DEM_TEST_I, 0x47);
    A7105_Strobe(A7105_STANDBY);
    A7105_WriteRegister(A7105_02_CALC,0x01);
    A7105_WriteRegister(A7105_0F_PLL_I,0x00);
    A7105_WriteRegister(A7105_02_CALC,0x02);
    A7105_WriteRegister(A7105_0F_PLL_I,0xA0);
    A7105_WriteRegister(A7105_02_CALC,0x02);
    A7105_Strobe(A7105_STANDBY);
}


void waitTRXCompletion(void)
{
    while(( A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK)) 
        ;
}

void strobeTXRX(void)
{
    A7105_WriteRegister(A7105_0F_PLL_I, channel);
    A7105_Strobe(A7105_TX);
    waitTRXCompletion();
    A7105_Strobe(A7105_RX);
    waitTRXCompletion();
    A7105_Strobe(A7105_RST_RDPTR);
}

void bind() 
{
    uint8_t chan=0;
  
    while(1){
   //     if( lib_timers_gettimermicroseconds(0) % 500000 > 250000)
   //         x4_set_leds(X4_LED_FR | X4_LED_RL);
   //     else
   //         x4_set_leds(X4_LED_FL | X4_LED_RR);

        A7105_Strobe(A7105_STANDBY);
        channel=allowed_ch[chan];
        if(chan==11)
            chan=0;
        A7105_WriteRegister(A7105_0F_PLL_I, channel);
        A7105_Strobe(A7105_RX);
        //unsigned long timer=lib_timers_starttimer();
        ms = 0;
        
        while(1){
            //if(lib_timers_gettimermicroseconds(timer) > 8000) {
            if (ms > 8)
            {
                chan++;
                break;
            }
            if(A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK){
                continue;
            }else{
                A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
                A7105_Strobe(A7105_RST_RDPTR);
                if (packet[0]==1){
                    break;
                } 
            }
        } 
        if (packet[0]==1){
            break;
        }
    }
    channel = packet[1];
  
    while(1) {
        hubsan_build_bind_packet(2);
        A7105_Strobe(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        strobeTXRX();
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0]==3){
            break;
        }
    }
  
    hubsan_build_bind_packet(4);
    A7105_Strobe(A7105_STANDBY);
    A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
    A7105_WriteRegister(A7105_0F_PLL_I, channel);
    A7105_Strobe(A7105_TX);
    waitTRXCompletion();

    A7105_ID_write(((uint32_t)packet[2] << 24) | ((uint32_t)packet[3] << 16) | ((uint32_t)packet[4] << 8) | packet[5]);
  
    while(1) { // useless block ?
        A7105_Strobe(A7105_RX);
        waitTRXCompletion();
        A7105_Strobe(A7105_RST_RDPTR);
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0]==1){
            break;
        }
    }
  
    while(1){
        hubsan_build_bind_packet(2);
        A7105_Strobe(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        strobeTXRX();
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0]==9){
            break;
        }
    }
  
    while(1){
        counter++;
        if(counter==10)
        counter=0;
        hubsan_build_bind_packet(0x0A);
        A7105_Strobe(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        strobeTXRX();
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (counter==9){
            break;
        }
    }
  
    A7105_WriteRegister(A7105_1F_CODE_I,0x0F); //CRC option CRC enabled adress 0x1f data 1111(CRCS=1,IDL=4bytes,PML[1:1]=4 bytes)
    //A7105_WriteRegister(0x28, 0x1F);//set Power to "1" dbm max value.
    A7105_Strobe(A7105_STANDBY);
    for(int i=0;i<4;i++){
        txid[i]=packet[i+11];
    }
}


void decodepacket()
{
    if(packet[0]==0x20) {
        // converts [0;255] to [-1;1] fixed point num
        ch[0] = 1116 + 3*packet[8];//ROLL
        ch[1] = 1116 + 3*packet[6];//PITCH
        ch[2] = 1116 + 3*packet[2]; //THROTTLE 
        ch[3] = 1116 + 3*packet[4]; //YAW
        ch[4] = (packet[9] & AUX1_FLAG) ? 1100 : 1900;//AUX1
        ch[5] = (packet[9] & AUX2_FLAG) ? 1100 : 1900;//AUX2
        
/*        lib_fp_lowpassfilter(&global.rxvalues[THROTTLEINDEX], ((fixedpointnum) packet[2] - 0x80) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        lib_fp_lowpassfilter(&global.rxvalues[YAWINDEX], ((fixedpointnum) packet[4] - 0x80) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        lib_fp_lowpassfilter(&global.rxvalues[PITCHINDEX], ((fixedpointnum) 0x80 - packet[6]) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        lib_fp_lowpassfilter(&global.rxvalues[ROLLINDEX], ((fixedpointnum) 0x80 - packet[8]) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        // "LEDs" channel, AUX1 (only on H107L, H107C, H107D and Deviation TXs, high by default)
        lib_fp_lowpassfilter(&global.rxvalues[AUX1INDEX], ((fixedpointnum) (packet[9] & AUX1_FLAG ? 0x7F : -0x7F)) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
        // "Flip" channel, AUX2 (only on H107L, H107C, H107D and Deviation TXs, high by default)
        lib_fp_lowpassfilter(&global.rxvalues[AUX2INDEX], ((fixedpointnum) (packet[9] & AUX2_FLAG ? 0x7F : -0x7F)) * 513L, global.timesliver, FIXEDPOINTONEOVERONESIXTYITH, TIMESLIVEREXTRASHIFT);
*/

    }
}

void readrx(void) // todo : telemetry
{
    if( ms > 14) {
        ms = 0;
        A7105_Strobe(A7105_RX);
    }
    if(A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK)
        return; // nothing received
    A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); 
    if(!((packet[11]==txid[0])&&(packet[12]==txid[1])&&(packet[13]==txid[2])&&(packet[14]==txid[3])))
        return; // not our TX !
    if(!hubsan_check_integrity())
        return; // bad checksum
    //timeout_timer = lib_timers_starttimer();
    ms = 0;
    A7105_Strobe(A7105_RST_RDPTR);
    A7105_Strobe(A7105_RX);
    decodepacket();
    // reset the failsafe timer
}


void loop()
{
  if (transmitting == 0)
  {
//    wait_for_read_rx = 1;

    readrx();
//        wait_for_read_rx = 0;
  }
}

