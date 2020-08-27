#include <msp430.h> 

#define READ_BUFFER_SIZE 12
#define ADC_BUFFER_SIZE 1024

#define LPM3_GIE() __bis_SR_register(LPM3_bits + GIE)

#define LPM0_GIE() __bis_SR_register(LPM0_bits + GIE)

const unsigned char AFI = 0x50; //Medical

unsigned short adc_buffer_write_index = 0;

unsigned char adc_buffer_last_read_index = 0;

unsigned char spi_counter = 0;

unsigned short spi_value = 0;

unsigned char timer_loop_counter = 0;

unsigned char request_transmit = 0;

unsigned short adc_buffer[ADC_BUFFER_SIZE];


unsigned char flag_inventory = 0;
unsigned char flag_select = 0;
unsigned char flag_address_mode = 0;
unsigned char host_uid[8];
const unsigned char info_flags = 0x6;
const unsigned char VICC_MAPPING_H = 0x0E;
const unsigned char VICC_MAPPING_L = 0x01;              //Block size of 2 bytes, 8 blocks

//Bit reverse table from: https://stackoverflow.com/a/2606268
static const unsigned char table[] = {
    0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
    0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
    0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
    0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
    0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
    0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
    0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
    0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
    0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
    0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
    0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
    0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
    0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
    0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
    0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
    0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
    0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
    0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
    0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
    0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
    0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
    0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
    0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
    0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
    0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
    0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
    0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
    0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
    0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
    0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
    0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
    0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
};

typedef enum TIMER_STATES_Enum{
    TRANSMIT,
    ACQUIRE,
} TIMER_STATES;

typedef enum RFTX_STATES_Enum{
    ACCUMULATE_B0,
    ACCUMULATE_B1,
} RFTX_STATES;

RFTX_STATES RFTX_STATE = ACCUMULATE_B0;
TIMER_STATES TIMER_STATE = ACQUIRE;

void configure_clocks(){
    //CCSCTL0
    CCSCTL0_H = 0xA5; // Unlocks the CCS registers

    //CCSCTL1
    //No change

    //CCSCTL4 Settings
    //All clocks set to LF-Oscillator
    //CCSCTL4 &= ~SELA0;

    //CCSCTL5 Settings
    CCSCTL5 &= ~DIVA__8;     //Clear DIVA0
    //CCSCTL5 |= DIVA__32;     //Set DIVA to 64kHz

    CCSCTL5 &= ~DIVS__8;     //Clear DIVA0
    //CCSCTL5 |= DIVS__32;     //Set DIVA to 64kHz

    CCSCTL5 &= ~DIVM__8;     //Clear DIVA0
    //CCSCTL5 |= DIVM__32;     //Set DIVA to 64kHz
    //Use the default values for S and M clock, they will be disabled later

    //CCSCTL6 Settings
    //XTOFF is disabled by default

    //CCSCTL7 Settings
    //No changes

    //CCSCTL8 Settings
    //No changes
}

void configure_SPI(){
    UCB0CTLW0 |= UCSWRST;

    //UCB0CTLW0 Settings
    UCB0CTLW0 |= UCMST;         //Set to master mode
    UCB0CTLW0 |= UCSSEL__ACLK;  //Set SPI to run off the ACLK
    UCB0CTLW0 |= UCMODE_0;      //Set to 3 wire spi
    UCB0BRW = 0xFE;
    //UCB0CTLW0 |= UCSTEM;        //Configure STE as chip select

    //UCB0BRW Settings
    //No changes

    //UCB0STATW Settings
    //No changes

    UCB0CTLW0 &= ~UCSWRST;                      //Clears the reset bit, SPI communication enabled

    UCB0IE |= UCRXIE;                           //Enable interrupts for RX
    //UCB0IE |= UCTXIE;
}

void configure_rfpmm(){
    //RFPMMCTL0 Settings
    //No changes

    //RFPMMCTL1 Settings
    //No changes
}

void configure_fram(){
    //FRCTL0 Settings
    //No Change

    //GCCTL0 Settings
    //No Change

    //GCCTL1 Settings
    //No Change
}

void configure_timer_a(){

    //TA0CTL = TAIE;

    //TA0CCTL Settings
    TA0CCTL0 = CCIE;
    TA0CCR0 = 64;

    //TA0CCTL Settings
    TA0CCTL1 = CCIE;
    TA0CCR1 = 1024;

    //TA0CTL Settings
    TA0CTL = TASSEL__ACLK | MC__CONTINUOUS | ID__1;     //Set timer from ACLK
}

void configure_rf(){
    //RF13MCTL Settings
    //No changes

    //RF13MINT Settings
    RF13MINT |= RX13MRFTOIE;    //Enable RF Timeout interrupt
    RF13MINT |= RX13MRXEIE;     //Enable RX error interupt
    RF13MINT |= RX13MOUFLIE;    //Enable overflow / underflow interrupt
    RF13MINT |= RF13MTXIE;      //TX done
    RF13MINT |= RF13MRXIE;      //RX done
    RF13MINT |= RF13MTXIFG;
    RF13MINT |= RF13MRXIFG;
    RF13MINT |= RF13MSLIFG;
    RF13MINT |= RF13MSLIE;
}

void configure_gpio(){
    //Configure all ports for default use (I/O), except 0 to 3, for SPI
    P1DIR |= BIT0;
    P1SEL1 &= ~BIT0;
    P1SEL0 |= BIT0;

    P1DIR |= BIT1;
    P1SEL1 &= ~BIT1;
    P1SEL0 |= BIT1;

    P1DIR |= BIT2;
    P1SEL1 &= ~BIT2;
    P1SEL0 |= BIT2;


    P1DIR |= BIT3;
    P1OUT |= BIT3;
    P1OUT |= BIT3;
}

/**
 * main.c
 */
int main(void)
{
	WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    configure_clocks();
    configure_gpio();
    configure_timer_a();
    configure_SPI();
    //configure_rfpmm();
    //configure_fram();
    configure_rf();

	adc_buffer_write_index = 0;

	__bis_SR_register(LPM0_bits + GIE);

	                                                    //Chip will be configured in LMP3 Mode, as the Timer used for initiating DAQ
	//LPM3_GIE();
}

__inline void parseFlag(unsigned char flag){
    flag_inventory = flag & 0x4;            //2nd bit
    if(!flag_inventory){
        flag_select = flag & 0x10;          //5th bit
        flag_address_mode = flag & 0x20;    //6th bit
    }
}

__inline sendUUID(){
    RF13MTXF_L = 0x01;
    RF13MTXF_L = 0x02;
    RF13MTXF_L = 0x03;
    RF13MTXF_L = 0x04;
    RF13MTXF_L = 0x05;
    RF13MTXF_L = 0x23;
    RF13MTXF_L = 0x02;  //TI
    RF13MTXF_L = 0xE0;
}

__inline unsigned char checkUUID(){
    unsigned char valid_address = 1;
    valid_address &= ((&RF13MRXBUF_L)[2] == 0x01);
    valid_address &= ((&RF13MRXBUF_L)[3] == 0x02);
    valid_address &= ((&RF13MRXBUF_L)[4] == 0x03);
    valid_address &= ((&RF13MRXBUF_L)[5] == 0x04);
    valid_address &= ((&RF13MRXBUF_L)[6] == 0x05);
    valid_address &= ((&RF13MRXBUF_L)[7] == 0x23);
    valid_address &= ((&RF13MRXBUF_L)[8] == 0x02);
    valid_address &= ((&RF13MRXBUF_L)[9] == 0xE0);
    return valid_address;
}

__inline void parse_iso_15693(){
    char x = 0;

    parseFlag((&RF13MRXBUF_L)[0]);

    unsigned char valid_address = 0;
    if(flag_address_mode){
        valid_address = checkUUID();
    }

    switch((&RF13MRXBUF_L)[1]){
    case 0x01:
        //Inventory
        RF13MTXF_L = 0x08; //No errors
        RF13MTXF_L = 0x00; //No DSFID
        sendUUID();
        break;

    case 0x2B:
        RF13MTXF_L = 0x08; //High bandwidth
        RF13MTXF_L = info_flags;
        sendUUID();
        RF13MTXF_L = AFI;
        RF13MTXF_L = VICC_MAPPING_L;
        RF13MTXF_L = VICC_MAPPING_H;
        break;

    case 0x20:
        if(valid_address){
            RF13MTXF_L = 0x08; //High bandwidth
            unsigned short checksum = 0;
            for(x = 0; x < READ_BUFFER_SIZE; x++){
                if(adc_buffer_last_read_index >= ADC_BUFFER_SIZE){
                    adc_buffer_last_read_index = 0;
                }else{
                    adc_buffer_last_read_index += 1;
                }

                if(adc_buffer_last_read_index < adc_buffer_write_index){
                    RF13MTXF = adc_buffer[adc_buffer_last_read_index];
                    checksum += adc_buffer[adc_buffer_last_read_index];
                }else{
                    break;
                }
            }
            RF13MTXF = checksum;
        }
        break;

    default:
        x = 1;
        break;
    }

    RF13MCTL |= RF13MTXEN;
}

#pragma vector=RF13M_VECTOR
__interrupt void RF_Int(void){
    switch(RF13MIV_L){
    case 2:
        //Data recieved
        request_transmit = 1;
        break;
    case 4:
        RF13MCTL |= RF13MRXEN;
        break;
    default:
        break;
    }
}

#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void){
    //Fire off the Acquire signal every 125Hz

    unsigned int iv = TA0IV;

    TA0CCR1 += 160;

    if(request_transmit && timer_loop_counter != 0){
        TIMER_STATE = TRANSMIT;
    }

    switch(iv){
    case 0x2:
        switch(TIMER_STATE){
        case ACQUIRE:
            if(timer_loop_counter == 0){
                RFTX_STATE = ACCUMULATE_B0;
                UCB0TXBUF_L = 1;
                spi_counter = 0;
                spi_value = 0;
                P1OUT &= ~BIT3;                                 //~CS -> low
                TIMER_STATE = ACQUIRE;
            }
            break;
        case TRANSMIT:
            parse_iso_15693();
            TIMER_STATE = ACQUIRE;
            request_transmit = 0;
            break;
        }
        timer_loop_counter += 1;
        if(timer_loop_counter >= 16){
            timer_loop_counter = 0;
        }
        break;
    }
}

#pragma vector=USCI_B0_VECTOR
__interrupt void SPI_ISR(void){

    switch(RFTX_STATE){
//    case CALIBRATE:
////        if(spi_counter < 8){
////            UCB0TXBUF_L = 1;
////            spi_counter += 1;
////        }else{
////            RFTX_STATE = ACCUMULATE_B0;
////        }
//        break;

    case ACCUMULATE_B0:
        spi_value = table[(&UCB0RXBUF_L)[0]];
        spi_value = spi_value << 8;
        UCB0TXBUF_L = 1;
        RFTX_STATE = ACCUMULATE_B1;
        break;

    case ACCUMULATE_B1:
        spi_value += table[(&UCB0RXBUF_L)[0]];
        adc_buffer[adc_buffer_write_index] = spi_value;
        adc_buffer_write_index += 1;
        if(adc_buffer_write_index >= ADC_BUFFER_SIZE){                        //Reset index if needed
            adc_buffer_write_index = 0;
        }
        P1OUT |= BIT3;
        break;
    }
}

#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void){

}

