/*
    Flybrix R/C Receiver Firmware -- Copyright 2015 Flying Selfie Inc.
    License and other details available at: http://www.flybrix.com/

    A7105 interface code based on https://github.com/mowfask/A7105-uart/tree/master/src/A7105-uart
    receiver protocol based on BradWii's implementation https://github.com/goebish/bradwii-X4/blob/master/src/rx_x4.c

*/
#include "common.h"

//#define DEBUG

extern "C" {
    #include "A7105_spi.h"
    #include "A7105.h"
}

void debug_byte_low(uint8_t b)
{
    PORTB &= 0xF7;
    __asm__("nop\n\t");

    for (uint8_t i = 0; i < 8; i++) {
        if ((b >> i) & 1) {
            PORTB |= 0x08;
        }
        else {
            PORTB &= 0xF7;
        }
        __asm__("nop\n\t");
        __asm__("nop\n\t");
        __asm__("nop\n\t");
        __asm__("nop\n\t");
        __asm__("nop\n\t");
    }
    __asm__("nop\n\t");
    PORTB |= 0x08;
}

void debug_spike_low()
{
    PORTB &= 0xF7;
    __asm__("nop\n\t");
    PORTB |= 0x08;
}

void debug_pulse_low()
{
    PORTB &= 0xF7;
    for (uint8_t i = 0; i < 10; i++) {
        __asm__("nop\n\t");
    }
    PORTB |= 0x08;
}

void debug_pulse_high()
{
    PORTB |= 0x08;
    for (uint8_t i = 0; i < 10; i++) {
        __asm__("nop\n\t");
    }
    PORTB &= 0xF7;
}

// renaming for clarity
#define A7105_WriteRegister SPI_reg_write
#define A7105_ReadRegister SPI_reg_read
#define A7105_WriteByte SPI_single_write

// global memory
static uint8_t packet[16]; // stores the latest packet of RX data
static uint8_t txid[4];    // keeps track of which transmitter we are listening to
static uint8_t green = 0x00;
static uint8_t red = 0x00;
static uint8_t blue = 0x04;

// timer 0 is used while binding and then turned off
volatile uint16_t ms = 0;
volatile uint8_t reset_flag = 1;

// wait to reset msec timer only on the interrupt to preserve accuracy
void reset_on_next_ms()
{
    reset_flag = 1;
}

ISR(TIM0_COMPA_vect)
{
    cli();
    if (reset_flag) {
        reset_flag = 0;
        ms = 0;
    }
    else {
        ms++;
    }
    sei();
}

//
// six channel PPM output waveform in usec
//
// ch0(low), ch0(high),
// ch1(low), ch1(high),
// ch2(low), ch2(high),
// ch3(low), ch3(high),
// ch4(low), ch4(high),
// ch5(low), ch5(high),
// sync(low),
// sync(high) <-- only update values during the long reset
//
volatile uint8_t cPPM_index = 0;
static uint8_t sync_pulse_interval_index = 0;
static uint8_t sync_pulse_interval_complete = 1;
volatile uint16_t channel_usec[6] = {1100, 1101, 1102, 1103, 1104, 1105}; //defaults
volatile uint16_t cPPM_usec[14] = {300, 800, 300, 801, 300, 802, 300, 803, 300, 804, 300, 805, 300, 8100}; //defaults
static const uint8_t PORTB_bit3[14] = {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1};
#define cPPM_sync 13 /* cPPM_usec should be updated only when we are in the final long reset stage */

ISR(TIM1_COMPA_vect)
{
    uint16_t target_delay_usec = 112;
    const uint16_t min_delay_usec = 56;
    if (cPPM_usec[cPPM_index] == 0) {
        cPPM_index++;
        if (cPPM_index > cPPM_sync) {
            //load new channel data at the end of the sync pulse
            uint16_t total_usec = 2100; // low pulses
            for (uint8_t i = 0; i < 6; i++) {
                cPPM_usec[2 * i    ] = 300; // channel low
                cPPM_usec[2 * i + 1] = channel_usec[i] - 300;
                total_usec += channel_usec[i];
            }
            cPPM_usec[cPPM_sync - 1] = 300; // sync low
            // pulse width should be minimum 2500usec
            // so total time should be > 15886
            cPPM_usec[cPPM_sync] = 18000 - total_usec;
            cPPM_index = 0;
        }
        if (PORTB_bit3[cPPM_index]) {
            PORTB |= 0x08;
        } else {
            PORTB &= 0xF7;
        }
    }
    // TCCR1 should be set up for 0.5usec/count
    if ( cPPM_usec[cPPM_index] > (target_delay_usec + min_delay_usec) ) {
        //wait full target delay
        cPPM_usec[cPPM_index] -= target_delay_usec;
    }
    else if ( cPPM_usec[cPPM_index] > target_delay_usec ) {
        //wait min delay
        target_delay_usec = min_delay_usec;
        cPPM_usec[cPPM_index] -= min_delay_usec;
    }
    else {
        //wait remainder >= min delay
        target_delay_usec = cPPM_usec[cPPM_index];
        cPPM_usec[cPPM_index] = 0;
    }
    if (cPPM_index == cPPM_sync) {
        sync_pulse_interval_index++; //advance to next interval work block
        sync_pulse_interval_complete = 0; //work is required during the pulse interval
    }
    OCR1A = (uint8_t)(target_delay_usec * 500 / 251);
    OCR1C = OCR1A;
}

void rgb(uint8_t r, uint8_t g, uint8_t b)
{
    red   = r;
    green = g;
    blue  = b;
}

void rgbFromIndex(uint8_t index)
{
    uint8_t l = 30;
    uint8_t h = 50;
    index = index % 16;
    if ( index & 0x04 ) { //high intensity
        red   = (index & 0x03) ? l : 0;
        green = (index & 0x02) ? l : 0;
        blue  = (index & 0x01) ? l : 0;
    }
    else { //low intensity
        red   = (index & 0x03) ? h : 2;
        green = (index & 0x02) ? h : 2;
        blue  = (index & 0x01) ? h : 2;
    }
}

static const uint8_t halfsine[180]={
  0,  9, 18, 27, 35, 44, 53, 62, 70, 79, 87, 96,104,112,120,128,135,143,
150,157,164,171,177,183,190,195,201,206,211,216,221,225,229,233,236,240,
243,245,247,249,251,253,254,254,255,255,255,254,254,253,251,249,247,245,
243,240,236,233,229,225,221,216,211,206,201,195,190,183,177,171,164,157,
150,143,135,128,120,112,104, 96, 87, 79, 70, 62, 53, 44, 35, 27, 18,  9,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0};

// sine wave rainbow
void rainbow(uint8_t halfangle)
{
  rgb(halfsine[(2*halfangle+120)%180], halfsine[2*halfangle],  halfsine[(2*halfangle+240)%180]);
}

#define PB4_HIGH_ASM "out 0x18, r6\n\t" /* match to h register */
#define PB4_LOW_ASM  "out 0x18, r7\n\t" /* match to l register */
#define FIVE_NOPS    "nop\n\t nop\n\t nop\n\t nop\n\t nop\n\t"
#define THREE_NOPS   "nop\n\t nop\n\t nop\n\t"

void updateLED()
{
    //send G7..G0 R7..R0 B7..B0
    volatile uint8_t h = (PORTB | 0x10);
    volatile uint8_t l = (PORTB & 0xEF);
    __asm__ ("mov r6, %0\n\t"::"r"(h));
    __asm__ ("mov r7, %0\n\t"::"r"(l));

    // registers 13+ seem to get clobbered!
    /*
    __asm__ (
        //"sbi 0x18, 0x4\n\t" //output high (2 ops)
        //"cbi 0x18, 0x4\n\t" //output low (2 ops)
        //"nop\n\t"
        PB4_HIGH_ASM
        PB4_LOW_ASM
    ::);
   */

    volatile uint8_t g = green;
    volatile uint8_t r = red;
    volatile uint8_t b = blue;

    volatile uint8_t c = 8;
    volatile uint8_t cr = 8;
    __asm__ ("mov r1, %0\n\t"::"r"(g));
    __asm__ ("mov r2, %0\n\t"::"r"(r));
    __asm__ ("mov r3, %0\n\t"::"r"(b));
    __asm__ ("mov r4, %0\n\t"::"r"(c));
    __asm__ ("mov r5, %0\n\t"::"r"(cr));
    __asm__ (
    "start_green:\n\t"
        PB4_HIGH_ASM
        THREE_NOPS
        "tst r1\n\t" //test green
        "brmi skip_green\n\t" //skip 1 instruction if r1[7] = 1
        PB4_LOW_ASM
        "skip_green:\n\t"
        FIVE_NOPS
        PB4_LOW_ASM
        FIVE_NOPS
        "lsl r1\n\t" //left shift green
        "dec r4\n\t"
        "breq end_green\n\t" // go to end_green when CNT == 0
            "nop\n\t"
            "rjmp start_green\n\t"
    "end_green:\n\t"
    "mov r4, r5\n\t" //reset count to 8
    "start_red:\n\t"
        PB4_HIGH_ASM
        THREE_NOPS
        "tst r2\n\t" //Test red
        "brmi skip_red\n\t" //skip 1 instruction if r2[7] = 1
        PB4_LOW_ASM
        "skip_red:\n\t"
        FIVE_NOPS
        PB4_LOW_ASM
        FIVE_NOPS
        "lsl r2\n\t" //left shift red
        "dec r4\n\t"
        "breq end_red\n\t" // go to end_red when CNT == 0
            "nop\n\t"
            "rjmp start_red\n\t"
    "end_red:\n\t"
    "mov r4, r5\n\t" //reset count to 8
    "start_blue:\n\t"
        PB4_HIGH_ASM
        THREE_NOPS
        "tst r3\n\t" //test blue
        "brmi skip_blue\n\t" //skip 1 instruction if r3[7] = 1
        PB4_LOW_ASM
        "skip_blue:\n\t"
        FIVE_NOPS
        PB4_LOW_ASM
        FIVE_NOPS
        "lsl r3\n\t" //left shift blue
        "dec r4\n\t"
        "breq end_blue\n\t" // go to end_blue when CNT == 0
            "nop\n\t"
            "rjmp start_blue\n\t"
    "end_blue:\n\t"
    PB4_LOW_ASM
    ::);
}

void sendIndexedColor(uint8_t index)
{
    rgbFromIndex(index);
    cli();
    updateLED();
    sei();
}

void createBindingPacket(uint8_t bindstate, uint8_t channel, uint8_t counter)
{
    packet[0] = bindstate;
    packet[1] = (bindstate != 0x0a) ? channel : counter;
    packet[6] = 0x08;
    packet[7] = 0xe4;
    packet[8] = 0xea;
    packet[9] = 0x9e;
    packet[10] = 0x50;
    //update the checksum
    uint16_t sum = 0;
    for (uint8_t i = 0; i < 15; i++) {
        sum += packet[i];
    }
    packet[15] = (256 - (sum % 256)) & 0xff;
}

uint8_t waitTRXCompletion(void)
{
    uint16_t max_attempts = 1000; //~50usec for each read
    while (max_attempts > 0) {
        if ( A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK ) {
            max_attempts--;
        }
        else {
            return 1;
        }
    }
    return 0;
}

//returns success or failure
uint8_t rebind()
{
    A7105_WriteByte(A7105_STANDBY);

    //set up timer 0 to count msec
    cli();
    TIMSK  = (1 << OCIE0A); //enable match A interrupt on TIM0 and turn off TIM1
    TCCR0A = (1 << WGM01); // clear timer on compare match
    TCCR0B = 0x03; // count at CLK/64 : ~4usec/tick
    OCR0A  = 249;  // tuned to 1 msec
    sei();

    A7105_reset();
    A7105_ID_write(0x55201041);
    A7105_WriteRegister(A7105_01_MODE_CONTROL, 0x63);
    A7105_WriteRegister(A7105_02_CALC, 0x00); //reset
    A7105_WriteRegister(A7105_03_FIFOI, 0x0F); //16 bytes
    A7105_WriteRegister(A7105_04_FIFOII, 0xC0);
    // GIO:
    //   0x01 -- WTR
    //   0x05 -- EOAC/FSYNC
    //   0x09 -- TMEO/CD
    //   0x0D -- PMDO
    A7105_WriteRegister(A7105_0B_GPIO1_PIN1, 0x05);
    A7105_WriteRegister(A7105_0C_GPIO2_PIN_II, 0x09);
    A7105_WriteRegister(A7105_0D_CLOCK, 0x05);
    A7105_WriteRegister(A7105_0E_DATA_RATE, 0x04);
    A7105_WriteRegister(A7105_15_TX_II, 0x2b);
    A7105_WriteRegister(A7105_18_RX, 0x62);
    A7105_WriteRegister(A7105_19_RX_GAIN_I, 0x80);
    A7105_WriteRegister(A7105_1C_RX_GAIN_IV, 0x0A);
    A7105_WriteRegister(A7105_1F_CODE_I, 0x07);
    A7105_WriteRegister(A7105_20_CODE_II, 0x17);
    A7105_WriteRegister(A7105_29_RX_DEM_TEST_I, 0x47);
    A7105_WriteByte(A7105_STANDBY);
    A7105_WriteRegister(A7105_02_CALC, 0x01);
    A7105_WriteRegister(A7105_0F_PLL_I, 0x00);
    A7105_WriteRegister(A7105_02_CALC, 0x02);
    A7105_WriteRegister(A7105_0F_PLL_I, 0xA0);
    A7105_WriteRegister(A7105_02_CALC, 0x02);
    A7105_WriteByte(A7105_STANDBY);

    // binding process
    uint8_t channel = 0;
    uint8_t counter = 0;
    const uint8_t allowed_ch[] = {0x14, 0x1E, 0x28, 0x32, 0x3C, 0x46, 0x50, 0x5A, 0x64, 0x6E, 0x78, 0x82};

    reset_on_next_ms();

    uint8_t channel_search_blink_state = 0;
    uint8_t found_channel = 0;
    while (!found_channel) { // scan through the PLL channels, ~10 msec at a time

        if (channel == 11) { // this code runs once every ~120 msec
            channel = 0;

            //blink LED on and off in orange
            switch (channel_search_blink_state) {
                case 1:
                    rgb(40,10,0);
                    channel_search_blink_state++;
                    break;
                case 6:
                    rgb(0,0,0);
                    channel_search_blink_state++;
                    break;
                case 10:
                    channel_search_blink_state = 0;
                    break;
                default:
                    channel_search_blink_state++;
                    break;
            }
            cli();
            updateLED();
            sei();
        }

        A7105_WriteByte(A7105_STANDBY);
        A7105_WriteRegister(A7105_0F_PLL_I, allowed_ch[channel]);
        A7105_WriteByte(A7105_RX);

        while (1) { // look for data on the current channel
            if (ms > 10) { // give up on this channel
                channel++;
                reset_on_next_ms();
                break;
            }
            if (A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK) {
                continue; //nothing received yet
            }
            else {
                A7105_WriteByte(A7105_RST_RDPTR);
                A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
                if (packet[0] == 1) {
                    found_channel = 1;
                    break;
                }
            }
        }
    }

    // we now know our PLL channel
    channel = packet[1];

    reset_on_next_ms();
    sendIndexedColor(channel);
    while (1) {    // show the user the channel id for 500 msec
        if (ms > 500) {
            reset_on_next_ms();
            break;
        }
    }
    // turn off the LED to indicate an errors if any of the remaining steps fail
    rgb(0, 0, 0);
    cli();
    updateLED();
    sei();

    reset_on_next_ms();
    while (1) {
        if (ms > 1000) {
            return 0;
        }
        createBindingPacket(2, channel, counter);
        A7105_WriteByte(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        A7105_WriteRegister(A7105_0F_PLL_I, channel);
        A7105_WriteByte(A7105_TX);
        if (!waitTRXCompletion()) {
            return 0;
        }
        A7105_WriteByte(A7105_RX);
        if (!waitTRXCompletion()) {
            return 0;
        }
        A7105_WriteByte(A7105_RST_RDPTR);
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0] == 3) {
            break;
        }
    }

    createBindingPacket(4, channel, counter);
    A7105_WriteByte(A7105_STANDBY);
    A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
    A7105_WriteRegister(A7105_0F_PLL_I, channel);
    A7105_WriteByte(A7105_TX);
    if (!waitTRXCompletion()) {
        return 0;
    }

    A7105_ID_write(((uint32_t)packet[2] << 24) | ((uint32_t)packet[3] << 16) | ((uint32_t)packet[4] << 8) | (uint32_t) packet[5]);

    reset_on_next_ms();
    while (1) { // useless block ?
        if (ms > 1000) {
            return 0;
        }
        A7105_WriteByte(A7105_RX);
        if (!waitTRXCompletion()) {
            return 0;
        }
        A7105_WriteByte(A7105_RST_RDPTR);
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0] == 1) {
            break;
        }
    }

    reset_on_next_ms();
    while (1) {
        if (ms > 1000) {
            return 0;
        }
        createBindingPacket(2, channel, counter);
        A7105_WriteByte(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        A7105_WriteRegister(A7105_0F_PLL_I, channel);
        A7105_WriteByte(A7105_TX);
        if (!waitTRXCompletion()) {
            return 0;
        }
        A7105_WriteByte(A7105_RX);
        if (!waitTRXCompletion()) {
            return 0;
        }
        A7105_WriteByte(A7105_RST_RDPTR);
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (packet[0] == 9) {
            break;
        }
    }

    reset_on_next_ms();
    while (1) {
        if (ms > 1000) {
            return 0;
        }
        counter++;
        if (counter == 10) {
            counter = 0;
        }
        createBindingPacket(0x0A, channel, counter);
        A7105_WriteByte(A7105_STANDBY);
        A7105_WritePayload((uint8_t*)&packet, sizeof(packet));
        A7105_WriteRegister(A7105_0F_PLL_I, channel);
        A7105_WriteByte(A7105_TX);
        if (!waitTRXCompletion()) {
            return 0;
        }
        A7105_WriteByte(A7105_RX);
        if (!waitTRXCompletion()) {
            return 0;
        }
        A7105_WriteByte(A7105_RST_RDPTR);
        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet));
        if (counter == 9) {
            break;
        }
    }

    A7105_WriteRegister(A7105_1F_CODE_I, 0x0F); //CRC option CRC enabled adress 0x1f data 1111(CRCS=1,IDL=4bytes,PML[1:1]=4 bytes)
    //A7105_WriteRegister(0x28, 0x1F);//set Power to "1" dbm max value.
    A7105_WriteByte(A7105_STANDBY);

    for (uint8_t i = 0; i < 4; i++) {
        txid[i] = packet[i + 11];
    }

    //binding was successful; return to indicating channel color
    sendIndexedColor(channel);

    A7105_WriteByte(A7105_RX);
    if (!waitTRXCompletion()) {
        return 0;
    }

    cli();
    GTCCR = 0x0; // PWM1B must be zero to turn off PWM mode
    TIMSK = (1 << OCIE1A); // turn off TIM0
    PORTB &= 0xF7; // start low (ch1 low pulse)
    TCCR1 = 128 | 4; // 0.5usec per count at 16Mhz clock
    OCR1A = 240;
    OCR1C = OCR1A;
    sei();

    return 1;
}

void setup()
{
    pinMode(4, OUTPUT);
    pinMode(3, OUTPUT);

    SPI_init();

    while (!rebind()) {
        continue;
    }
}

#ifdef DEBUG
// test waveforms
static uint8_t tstep = 0;
#endif

static uint8_t cycle_completed = 0;
static uint8_t missed_cycles = 0;
static uint8_t packet_ready = 0;

void loop() {
    if (!cycle_completed) { //this cycle is not yet finished
        if ((cPPM_index == cPPM_sync) && (!sync_pulse_interval_complete)) {
            switch ( sync_pulse_interval_index ) { //~120 usec per case
                case 1:
                    if (A7105_ReadRegister(A7105_00_MODE) & A7105_MODE_TRER_MASK) { //still busy!
                        missed_cycles++; // packet wasn't ready
                        packet_ready = 0;
                    }
                    else {
                        A7105_WriteByte(A7105_RST_RDPTR);
                        A7105_ReadPayload((uint8_t*)&packet, sizeof(packet)); //~600usec!
                        packet_ready = 1;
                    }
                    break;

                // interrupt callback moves us forward several indices while we read data (~600usec)

                case 11:
                    #ifdef DEBUG
                        //update test patterns
                        for (uint8_t i = 0; i < 6; i++) {
                            channel_usec[i] = 1100 + i + 255 / 30 * abs(90 - tstep);
                        }
                        tstep++;
                        if (tstep == 180) {
                            tstep=0;
                        rainbow(tstep);
                    #endif
                    break;

                case 12:
                    if (packet_ready) {
                        if (!((packet[11] == txid[0]) && (packet[12] == txid[1]) && (packet[13] == txid[2]) && (packet[14] == txid[3]))) {
                            missed_cycles++; // not our TX !
                            packet_ready = 0;
                        }
                    }
                    break;

                case 13:
                    if (packet_ready) {
                        uint16_t sum = 0;
                        for (uint8_t i = 0; i < 15; i++) {
                            sum += packet[i];
                        }
                        if (!(packet[15] == ((256 - (sum % 256)) & 0xff))) {
                            missed_cycles++; //bad checksum
                            packet_ready = 0;
                        }
                    }
                    break;

                case 14:
                    if (packet_ready) {
                        // decode the packet
                        if (packet[0] == 0x20) {
                            // converts [0;255] to [-1;1] fixed point num
                            channel_usec[0] = 1116 + 3 * packet[8]; //ROLL
                            channel_usec[1] = 1116 + 3 * packet[6]; //PITCH
                            channel_usec[2] = 1116 + 3 * packet[2]; //THROTTLE
                            channel_usec[3] = 1116 + 3 * packet[4]; //YAW
                            channel_usec[4] = (packet[9] & 0x04) ? 1881 : 1116; //AUX1 (left trim hold)
                            channel_usec[5] = (packet[9] & 0x08) ? 1116 : 1881; //AUX2 (left stick press)
                            if (missed_cycles) {
                                missed_cycles--;
                            }
                        }
                        else {
                            missed_cycles++; // bad packet[0]
                        }
                    }
                    packet_ready = 0;
                    break;

                case 15:
                    {   //new context so we can declare variables
                        uint8_t green_level = (missed_cycles > 5) ? 10 : (5 + missed_cycles);
                        uint8_t red_level = missed_cycles;
                        rgb(red_level, green_level, 0);
                    }
                    break;

                case 16:
                    cli();
                    updateLED();
                    sei();
                    break;

                case 17:
                    A7105_WriteByte(A7105_RST_RDPTR); //reset the FIFO read pointer
                    A7105_WriteByte(A7105_RX); //RX strobe
                    cycle_completed = 1;
                    break;

                default:
                    break;
            }//end switch
            sync_pulse_interval_complete = 1;
        }
    }
    else { // this cycle is finished but we're still finishing up the sync pulse
        if (cPPM_index != cPPM_sync) { // set up for next cycle when exiting sync
            sync_pulse_interval_index = 0;
            cycle_completed = 0;

            // send a few cycles to indicate distress before we give up and rebind
            if ( missed_cycles > 45 ) {
                channel_usec[0] = 1000; //ROLL
                channel_usec[1] = 1000; //PITCH
                channel_usec[2] = 1000; //THROTTLE
                channel_usec[3] = 1000; //YAW
                channel_usec[4] = 1900; //AUX1 (left trim hold)
                channel_usec[5] = 1000; //AUX2 (left stick press)
            }
            
            if ( missed_cycles > 50 ) {
                while (!rebind()) {
                    continue;
                }
                missed_cycles = 0;
                packet_ready = 0;
            }
        }
    }
}

