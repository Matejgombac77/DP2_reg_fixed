/**************************************************************
* FILE:         ADC_drv.c
* DESCRIPTION:  A/D driver for piccolo devices
* AUTHOR:       Mitja Nemec
* DATE:         19.1.2012
*
****************************************************************/
#include "ADC_drv.h"

/**************************************************************
* inicializiramo ADC
**************************************************************/
void ADC_init(void)
{
    // *IMPORTANT*
    // The Device_cal function, which copies the ADC calibration values from TI reserved
    // OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
    // Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC to function according
    // to specification. The clocks to the ADC MUST be enabled before calling this
    // function.
    // See the device data manual and/or the ADC Reference
    // Manual for more information.

    EALLOW;
    SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
    (*Device_cal)();
    EDIS;

    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
    // after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
    // CPU_RATE define statement in the DSP2802x_Examples.h file must
    // contain the correct CPU clock period in nanoseconds.
    EALLOW;
    AdcRegs.ADCCTL1.bit.ADCBGPWD = 1;       // Power ADC BG
    AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;      // Power reference
    AdcRegs.ADCCTL1.bit.ADCPWDN = 1;        // Power ADC - vklop analognega vezja znotraj procesorja
    AdcRegs.ADCCTL1.bit.ADCENABLE = 1;      // Enable ADC - omogocimo delovanje adc-ja
    AdcRegs.ADCCTL1.bit.ADCREFSEL = 0;      // Select interal BG - izberemo notranjo referenco
    AdcRegs.ADCCTL1.bit.TEMPCONV = 1;       // izberemo notranji temperaturni senzor na adcin5 vhodu
    AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    EDIS;

    DELAY_US(1000L);         // Delay before converting ADC channels

    // Configure ADC
    EALLOW;
    
    // ADC trigger setup
    ADC_MODUL1.CMPB = 0x0000;
    ADC_MODUL1.ETSEL.bit.SOCASEL = ET_CTR_ZERO;   //sproži prekinitev na periodo
    ADC_MODUL1.ETPS.bit.SOCAPRD = ET_1ST;        //ob vsakem prvem dogodku
    ADC_MODUL1.ETCLR.bit.SOCA = 1;               //clear possible flag
    ADC_MODUL1.ETSEL.bit.SOCAEN = 1;             //enable ADC Start Of conversion

    // SOC0 config
    AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x1;     //set SOC0 channel select to ADCINA1
    AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05;  //set SOC0 to start trigger on EPWM1A
    AdcRegs.ADCSOC0CTL.bit.ACQPS = 6;       //set SOC0 S/H Window to 7 ADC Clock Cycles,

    // SOC1 config
    AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x1;     //set SOC1 channel select to ADCINA1
    AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05;  //set SOC1 to start trigger on EPWM1A
    AdcRegs.ADCSOC1CTL.bit.ACQPS = 6;       //set SOC1 S/H Window to 7 ADC Clock Cycles,

    // SOC2 config
    AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x0;     //set SOC2 channel select to ADCINA0
    AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0x05;  //set SOC2 to start trigger on EPWM1A
    AdcRegs.ADCSOC2CTL.bit.ACQPS = 6;       //set SOC2 S/H Window to 7 ADC Clock Cycles,

    // SOC3 config
    AdcRegs.ADCSOC3CTL.bit.CHSEL = 0x2;     //set SOC3 channel select to ADCINA2
    AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 0x05;  //set SOC3 to start trigger on EPWM1A
    AdcRegs.ADCSOC3CTL.bit.ACQPS = 6;       //set SOC3 S/H Window to 7 ADC Clock Cycles,

    //tu povemo naj se postavi interrupt flag, ko je zadnja pretvorba koncna
    //interrupt je se naprej onemogocen, flag ki se bo postavil pa nam bo
    //sluzil za detektiranje konca niza pretvorb
    AdcRegs.INTSEL1N2.bit.INT1SEL = 0x00;   //interrupt1 naj prozi signal EOC1, ker je to soc, ki se zadnji izvede
    AdcRegs.INTSEL1N2.bit.INT1E = 1;        //prekinitev ob interrpt dogodku je omogocena (da se lahko postavi flag)
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;   //pobrise se flag

    EDIS;

}   //end of AP_ADC_init

/**************************************************************
* Funkcija, ki pocaka da ADC konca s pretvorbo
* vzorcimo...
* return: void
**************************************************************/
void ADC_wait(void)
{
    while (AdcRegs.ADCINTFLG.bit.ADCINT1 == 0)
    {
        /* DO NOTHING */
    }
    AdcRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;               //pobrisem flag bit od ADC-ja
}
