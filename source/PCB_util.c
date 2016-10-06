/**************************************************************
* FILE:         PCB_util.c 
* DESCRIPTION:  PCB initialization & Support Functions
* VERSION:      1.0
* AUTHOR:       Mitja Nemec
**************************************************************/
#include "PCB_util.h"

/**************************************************************
* Funckija ki prizge LED diodo 1
**************************************************************/
void PCB_LED_on(void)
{
	GpioDataRegs.GPBSET.bit.GPIO34 = 1;
}

/**************************************************************
* Funckija ki ugasne LED diodo 1
**************************************************************/
void PCB_LED_off(void)
{
	GpioDataRegs.GPBCLEAR.bit.GPIO34 = 1;
}

/**************************************************************
* Funckija ki spremeni stanje LED diode
**************************************************************/
void PCB_LED_toggle(void)
{
	GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
}


/**************************************************************
* Funckija ki vklopi breme
**************************************************************/
void PCB_load_on(void)
{
	GpioDataRegs.GPADAT.bit.GPIO1 = 1;
}

/**************************************************************
* Funckija ki izklopi breme
**************************************************************/
void PCB_load_off(void)
{
	GpioDataRegs.GPADAT.bit.GPIO1 = 0;
}

/**************************************************************
* Funckija ki inicializira PCB
**************************************************************/
void PCB_init(void)
{
    EALLOW;

    /* IZHODI */
    // LED na sticku
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;
    GpioDataRegs.GPBDAT.bit.GPIO34 = 0;

    // BREME
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
    GpioDataRegs.GPADAT.bit.GPIO1 = 0;

    EDIS;

}
