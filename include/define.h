/****************************************************************
* FILENAME:     define.h           
* DESCRIPTION:  file with global define macros
* AUTHOR:       Mitja Nemec
* START DATE:   16.1.2009
* VERSION:      1.0
*
* CHANGES : 
* VERSION   DATE        WHO         DETAIL 
* 1.0       16.1.2009   Mitja Nemec Initial version
*
****************************************************************/
#ifndef     __DEFINE_H__
#define     __DEFINE_H__

#include    "IQmathLib.h"

// frekvenca PWM-ja
#define     SWITCH_FREQ     40000

// veèkratnik preklopne frekvence
#define     SAMP_PRESCALE   1

// Vzorèna frekvenca
#define     SAMPLE_FREQ     (SWITCH_FREQ/SAMP_PRESCALE)

// vzorèna perioda
#define     SAMPLE_TIME     (1.0/SAMPLE_FREQ)

// frekvenca omrežja (Hz)
#define     GRID_FREQ       50

// frekvenca procesorja v Hz
#define     CPU_FREQ        80000000

// definicije matematiènih konstant
#define     SQRT3           1.7320508075688772935274463415059
#define     SQRT2           1.4142135623730950488016887242097
#define     PI              3.1415926535897932384626433832795

// deklaracije za logicne operacije
typedef enum {FALSE = 0, TRUE} bool;

// kako naj se obnašajo preriferne naprave, ko ustavimo izvajanje programa
// 0 stop immediately, 1 stop when finished, 2 run free
#define     DEBUG_STOP      0

#endif // end of __DEFINE_H__ definition
