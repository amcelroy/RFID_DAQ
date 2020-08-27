/* ============================================================================ */
/* Copyright (c) 2020, Texas Instruments Incorporated                           */
/*  All rights reserved.                                                        */
/*                                                                              */
/*  Redistribution and use in source and binary forms, with or without          */
/*  modification, are permitted provided that the following conditions          */
/*  are met:                                                                    */
/*                                                                              */
/*  *  Redistributions of source code must retain the above copyright           */
/*     notice, this list of conditions and the following disclaimer.            */
/*                                                                              */
/*  *  Redistributions in binary form must reproduce the above copyright        */
/*     notice, this list of conditions and the following disclaimer in the      */
/*     documentation and/or other materials provided with the distribution.     */
/*                                                                              */
/*  *  Neither the name of Texas Instruments Incorporated nor the names of      */
/*     its contributors may be used to endorse or promote products derived      */
/*     from this software without specific prior written permission.            */
/*                                                                              */
/*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" */
/*  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,       */
/*  THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR      */
/*  PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR            */
/*  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,       */
/*  EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,         */
/*  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; */
/*  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,    */
/*  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR     */
/*  OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,              */
/*  EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                          */
/* ============================================================================ */

/******************************************************************************/
/* rf430frl152h.cmd                                                           */
/*    - Linker Command File for defintions in the header file                 */
/*    Please do not change !                                                  */
/*                                                                            */
/******************************************************************************/
/* Version: 1.209                                                             */
/******************************************************************************/


/************************************************************
* STANDARD BITS
************************************************************/
/************************************************************
* STATUS REGISTER BITS
************************************************************/
/************************************************************
* PERIPHERAL FILE MAP
************************************************************/
/************************************************************
* COMPACT CLOCK SYSTEM
************************************************************/
CCSCTL0            = 0x0160;
CCSCTL0_L          = 0x0160;
CCSCTL0_H          = 0x0161;
CCSCTL1            = 0x0162;
CCSCTL1_L          = 0x0162;
CCSCTL1_H          = 0x0163;
CCSCTL2            = 0x0164;
CCSCTL2_L          = 0x0164;
CCSCTL2_H          = 0x0165;
CCSCTL4            = 0x0168;
CCSCTL4_L          = 0x0168;
CCSCTL4_H          = 0x0169;
CCSCTL5            = 0x016A;
CCSCTL5_L          = 0x016A;
CCSCTL5_H          = 0x016B;
CCSCTL6            = 0x016C;
CCSCTL6_L          = 0x016C;
CCSCTL6_H          = 0x016D;
CCSCTL7            = 0x016E;
CCSCTL7_L          = 0x016E;
CCSCTL7_H          = 0x016F;
CCSCTL8            = 0x0170;
CCSCTL8_L          = 0x0170;
CCSCTL8_H          = 0x0171;
/*************************************************************
* CRC Module
*************************************************************/
CRCDI              = 0x0150;
CRCDI_L            = 0x0150;
CRCDI_H            = 0x0151;
CRCDIRB            = 0x0152;
CRCDIRB_L          = 0x0152;
CRCDIRB_H          = 0x0153;
CRCINIRES          = 0x0154;
CRCINIRES_L        = 0x0154;
CRCINIRES_H        = 0x0155;
CRCRESR            = 0x0156;
CRCRESR_L          = 0x0156;
CRCRESR_H          = 0x0157;
/************************************************************
* DIGITAL I/O Port1/2 Pull up / Pull down Resistors
************************************************************/
PAIN               = 0x0200;
PAIN_L             = 0x0200;
PAIN_H             = 0x0201;
PAOUT              = 0x0202;
PAOUT_L            = 0x0202;
PAOUT_H            = 0x0203;
PADIR              = 0x0204;
PADIR_L            = 0x0204;
PADIR_H            = 0x0205;
PAREN              = 0x0206;
PAREN_L            = 0x0206;
PAREN_H            = 0x0207;
PADS               = 0x0208;
PADS_L             = 0x0208;
PADS_H             = 0x0209;
PASEL0             = 0x020A;
PASEL0_L           = 0x020A;
PASEL0_H           = 0x020B;
PASEL1             = 0x020C;
PASEL1_L           = 0x020C;
PASEL1_H           = 0x020D;
PAIES              = 0x0218;
PAIES_L            = 0x0218;
PAIES_H            = 0x0219;
PAIE               = 0x021A;
PAIE_L             = 0x021A;
PAIE_H             = 0x021B;
PAIFG              = 0x021C;
PAIFG_L            = 0x021C;
PAIFG_H            = 0x021D;
P1IV               = 0x020E;
P2IV               = 0x021E;
/************************************************************
* RF13M (13MHz NFC communication module)
************************************************************/
RF13MCTL           = 0x0800;
RF13MCTL_L         = 0x0800;
RF13MCTL_H         = 0x0801;
RF13MINT           = 0x0802;
RF13MINT_L         = 0x0802;
RF13MINT_H         = 0x0803;
RF13MIV            = 0x0804;
RF13MIV_L          = 0x0804;
RF13MIV_H          = 0x0805;
RF13MRXF           = 0x0806;
RF13MRXF_L         = 0x0806;
RF13MRXF_H         = 0x0807;
RF13MTXF           = 0x0808;
RF13MTXF_L         = 0x0808;
RF13MTXF_H         = 0x0809;
RF13MCRC           = 0x080A;
RF13MCRC_L         = 0x080A;
RF13MCRC_H         = 0x080B;
RF13MFIFOFL        = 0x080C;
RF13MFIFOFL_L      = 0x080C;
RF13MFIFOFL_H      = 0x080D;
RF13MWMCFG         = 0x080E;
RF13MWMCFG_L       = 0x080E;
RF13MWMCFG_H       = 0x080F;
RF13MRXBUF         = 0x0820;
RF13MRXBUF_L       = 0x0820;
RF13MRXBUF_H       = 0x0821;
RF13MTXBUF         = 0x0840;
RF13MTXBUF_L       = 0x0840;
RF13MTXBUF_H       = 0x0841;
/************************************************************
* SD14 (14 Bit sigma delta ADC)
************************************************************/
SD14CTL0           = 0x0700;
SD14CTL0_L         = 0x0700;
SD14CTL0_H         = 0x0701;
SD14CTL1           = 0x0702;
SD14CTL1_L         = 0x0702;
SD14CTL1_H         = 0x0703;
SD14MEM0           = 0x0704;
SD14MEM0_L         = 0x0704;
SD14MEM0_H         = 0x0705;
SD14MEM1           = 0x0706;
SD14MEM1_L         = 0x0706;
SD14MEM1_H         = 0x0707;
SD14MEM2           = 0x0708;
SD14MEM2_L         = 0x0708;
SD14MEM2_H         = 0x0709;
SD14MEM3           = 0x070A;
SD14MEM3_L         = 0x070A;
SD14MEM3_H         = 0x070B;
SD14IV             = 0x070C;
SD14IV_L           = 0x070C;
SD14IV_H           = 0x070D;
/************************************************************
* USCI B0
************************************************************/
UCB0CTLW0          = 0x0640;
UCB0CTLW0_L        = 0x0640;
UCB0CTLW0_H        = 0x0641;
UCB0CTLW1          = 0x0642;
UCB0CTLW1_L        = 0x0642;
UCB0CTLW1_H        = 0x0643;
UCB0BRW            = 0x0646;
UCB0BRW_L          = 0x0646;
UCB0BRW_H          = 0x0647;
UCB0STATW          = 0x0648;
UCB0STATW_L        = 0x0648;
UCB0STATW_H        = 0x0649;
UCB0TBCNT          = 0x064A;
UCB0TBCNT_L        = 0x064A;
UCB0TBCNT_H        = 0x064B;
UCB0RXBUF          = 0x064C;
UCB0RXBUF_L        = 0x064C;
UCB0RXBUF_H        = 0x064D;
UCB0TXBUF          = 0x064E;
UCB0TXBUF_L        = 0x064E;
UCB0TXBUF_H        = 0x064F;
UCB0I2COA0         = 0x0654;
UCB0I2COA0_L       = 0x0654;
UCB0I2COA0_H       = 0x0655;
UCB0I2COA1         = 0x0656;
UCB0I2COA1_L       = 0x0656;
UCB0I2COA1_H       = 0x0657;
UCB0I2COA2         = 0x0658;
UCB0I2COA2_L       = 0x0658;
UCB0I2COA2_H       = 0x0659;
UCB0I2COA3         = 0x065A;
UCB0I2COA3_L       = 0x065A;
UCB0I2COA3_H       = 0x065B;
UCB0ADDRX          = 0x065C;
UCB0ADDRX_L        = 0x065C;
UCB0ADDRX_H        = 0x065D;
UCB0ADDMASK        = 0x065E;
UCB0ADDMASK_L      = 0x065E;
UCB0ADDMASK_H      = 0x065F;
UCB0I2CSA          = 0x0660;
UCB0I2CSA_L        = 0x0660;
UCB0I2CSA_H        = 0x0661;
UCB0IE             = 0x066A;
UCB0IE_L           = 0x066A;
UCB0IE_H           = 0x066B;
UCB0IFG            = 0x066C;
UCB0IFG_L          = 0x066C;
UCB0IFG_H          = 0x066D;
UCB0IV             = 0x066E;
/************************************************************
* SFR - Special Function Register Module
************************************************************/
SFRIE1             = 0x0100;
SFRIE1_L           = 0x0100;
SFRIE1_H           = 0x0101;
SFRIFG1            = 0x0102;
SFRIFG1_L          = 0x0102;
SFRIFG1_H          = 0x0103;
SFRRPCR            = 0x0104;
SFRRPCR_L          = 0x0104;
SFRRPCR_H          = 0x0105;
/************************************************************
* COMPACT SYS - System Module
************************************************************/
SYSCTL             = 0x0180;
SYSCTL_L           = 0x0180;
SYSCTL_H           = 0x0181;
SYSJMBC            = 0x0186;
SYSJMBC_L          = 0x0186;
SYSJMBC_H          = 0x0187;
SYSJMBI0           = 0x0188;
SYSJMBI0_L         = 0x0188;
SYSJMBI0_H         = 0x0189;
SYSJMBI1           = 0x018A;
SYSJMBI1_L         = 0x018A;
SYSJMBI1_H         = 0x018B;
SYSJMBO0           = 0x018C;
SYSJMBO0_L         = 0x018C;
SYSJMBO0_H         = 0x018D;
SYSJMBO1           = 0x018E;
SYSJMBO1_L         = 0x018E;
SYSJMBO1_H         = 0x018F;
SYSCNF             = 0x0190;
SYSCNF_L           = 0x0190;
SYSCNF_H           = 0x0191;
SYSBERRIV          = 0x0198;
SYSBERRIV_L        = 0x0198;
SYSBERRIV_H        = 0x0199;
SYSUNIV            = 0x019A;
SYSUNIV_L          = 0x019A;
SYSUNIV_H          = 0x019B;
SYSSNIV            = 0x019C;
SYSSNIV_L          = 0x019C;
SYSSNIV_H          = 0x019D;
SYSRSTIV           = 0x019E;
SYSRSTIV_L         = 0x019E;
SYSRSTIV_H         = 0x019F;
/************************************************************
* PMM_RF - Power Management System of RF
************************************************************/
RFPMMCTL0          = 0x0120;
RFPMMCTL0_L        = 0x0120;
RFPMMCTL0_H        = 0x0121;
RFPMMCTL1          = 0x0122;
RFPMMCTL1_L        = 0x0122;
RFPMMCTL1_H        = 0x0123;
RFMMIE             = 0x0124;
RFMMIE_L           = 0x0124;
RFMMIE_H           = 0x0125;
RFPMMIFG           = 0x0126;
RFPMMIFG_L         = 0x0126;
RFPMMIFG_H         = 0x0127;
RFPMMIV            = 0x0128;
RFPMMIV_L          = 0x0128;
RFPMMIV_H          = 0x0129;
/************************************************************
* Timer0_A3
************************************************************/
TA0CTL             = 0x0340;
TA0CCTL0           = 0x0342;
TA0CCTL1           = 0x0344;
TA0CCTL2           = 0x0346;
TA0R               = 0x0350;
TA0CCR0            = 0x0352;
TA0CCR1            = 0x0354;
TA0CCR2            = 0x0356;
TA0IV              = 0x036E;
TA0EX0             = 0x0360;
/************************************************************
* WATCHDOG TIMER A
************************************************************/
WDTCTL             = 0x015C;
WDTCTL_L           = 0x015C;
WDTCTL_H           = 0x015D;
/************************************************************
* Interrupt Vectors (offset from 0xFFFF - 0x20)
************************************************************/
/************************************************************
* End of Modules
************************************************************/
