/**************************************************************
* FILE:         PCB_util.h 
* DESCRIPTION:  definitions for PCB Initialization & Support Functions
* VERSION:      1.0
* AUTHOR:       Mitja Nemec
**************************************************************/
#ifndef   PCB_UTIL_H
#define   PCB_UTIL_H

#include    "DSP28x_Project.h"
#include    "define.h"



/**************************************************************
* Funckija ki prizge LED diodo 1
**************************************************************/
extern void PCB_LED_on(void);

/**************************************************************
* Funckija ki ugasne LED diodo 1
**************************************************************/
extern void PCB_LED_off(void);

/**************************************************************
* Funckija ki spremeni stanje LED diodo 1
**************************************************************/
extern void PCB_LED_toggle(void);

/**************************************************************
* Funckija ki izklopi breme
**************************************************************/
extern void PCB_load_off(void);

/**************************************************************
* Funckija ki vklopi breme
**************************************************************/
extern void PCB_load_on(void);

/**************************************************************
* Funckija ki inicializa MCU in tiskanino
**************************************************************/
extern void PCB_init(void);


#endif  // end of PCB_UTIL_H definition

