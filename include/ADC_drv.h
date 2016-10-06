/**************************************************************
* FILE:         ADC_drv.c
* DESCRIPTION:  A/D driver for piccolo devices
* AUTHOR:       Mitja Nemec
* DATE:         19.1.2012
*
****************************************************************/
#ifndef     __ADC_DRV_H__
#define     __ADC_DRV_H__

#include    "DSP28x_Project.h"

// doloci kateri PWM modul prozi ADC
#define     ADC_MODUL1      EPwm1Regs

// kje se nahajajo rezultati
#define     ADC_RESULT0     (AdcResult.ADCRESULT0)
#define     NAPETOST_ZA     (AdcResult.ADCRESULT1)
#define     NAPETOST_PRED   (AdcResult.ADCRESULT2)
#define     TOK             (AdcResult.ADCRESULT3)

/**************************************************************
* inicializiramo ADC
**************************************************************/
extern void ADC_init(void);

/**************************************************************
* Funkcija, ki pocaka da ADC konca s pretvorbo
* vzorcimo...
* return: void
**************************************************************/
extern void ADC_wait(void);

#endif /* __ADC_DRV_H__ */
