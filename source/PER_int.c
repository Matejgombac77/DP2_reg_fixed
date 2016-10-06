/****************************************************************
* FILENAME:     PER_int.c
* DESCRIPTION:  periodic interrupt code
* AUTHOR:       Mitja Nemec
* DATE:         16.1.2009
*
****************************************************************/
#include    "PER_int.h"
#include    "TIC_toc.h"

// definicija formata zapisa
#define     FORMAT      20      // F12.20

// za izracunun napetostni
long napetost_raw = 0;
long napetost_offset = 0;
long napetost_gain = ???;
long napetost = 0;

// za izracun toka
long tok_raw = 0;
long tok_offset = 2048;
long tok_gain = ???;
long tok = 0;

// vklopno razmerje
long duty = 0;

// generiranje željene vrednosti
long ref_counter = 0;
long ref_counter_prd = SWITCH_FREQ;
long ref_counter_cmpr = 800;
long ref_counter_load_on = 350;
long ref_counter_load_off = 650;

float ref_value = 0;
float ref_value_high = 2.5;
float ref_value_low = 0.25;

// za kalibracijo preostale napetosti tokovne sonde
bool    offset_calib = TRUE;
long     offset_counter = 0;
float   offset_filter = 0.005;

// za oceno obremenjenosti CPU-ja
float   cpu_load  = 0.0;
long    interrupt_cycles = 0;

/**************************************************************
* spremenljivke, ki jih potrebujemo za regulator
**************************************************************/
long zeljena = 0;







// spremenljikva s katero štejemo kolikokrat se je prekinitev predolgo izvajala
int interrupt_overflow_counter = 0;


/**************************************************************
* Prekinitev, ki v kateri se izvaja regulacija
**************************************************************/
#pragma CODE_SECTION(PER_int, "ramfuncs");
void interrupt PER_int(void)
{
    /* lokalne spremenljivke */
    
    // najprej povem da sem se odzzval na prekinitev
    // Spustimo INT zastavico casovnika ePWM1
    EPwm1Regs.ETCLR.bit.INT = 1;
    // Spustimo INT zastavico v PIE enoti
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    
    // pozenem stoprico
    interrupt_cycles = TIC_time;
    TIC_start();

    // izracunam obremenjenost procesorja
    cpu_load = (float)interrupt_cycles / (CPU_FREQ/SWITCH_FREQ);

    // pocakam da ADC konca s pretvorbo
    ADC_wait();

    // ali kalibriram preostalo napetost tokovne sonde
    if (offset_calib == TRUE)
    {
        tok_raw = TOK;
        tok_offset = (1.0 - offset_filter) * tok_offset + offset_filter * tok_raw;

        offset_counter = offset_counter + 1;
        if ( offset_counter == SWITCH_FREQ)
        {
            offset_calib = FALSE;
        }

    }
    // sicer pa normalno obratujem
    else
    {
        // preracunam napetost
        napetost_raw = NAPETOST_ZA;
        napetost = napetost_raw * napetost_gain;

        tok_raw = TOK - tok_offset;
        tok = -tok_raw * tok_gain;

        // generiram zeljeno vrednost
        ref_counter = ref_counter + 1;
        if (ref_counter >= ref_counter_prd)
        {
            ref_counter = 0;
        }
        
        // stopnica bremena
        if (   (ref_counter > ref_counter_load_on)
            && (ref_counter < ref_counter_load_off))
        {
            PCB_load_on();
        }
        if (   (ref_counter < ref_counter_load_on)
            || (ref_counter > ref_counter_load_off))
        {
            PCB_load_off();
        }
        
        // stopnicasta zeljena vrednost
        if (ref_counter > ref_counter_cmpr)
        {
            ref_value = ref_value_low;
        }
        else
        {
            ref_value = ref_value_high;
        }

        zeljena = ref_value * (1L << FORMAT);
        /*******************************************************
        * Tukaj pride koda regulatorja
        *******************************************************/

        
        
        
        
        
        
        
        
        
        
        // osvežim vklono razmerje
        PWM_update((float)duty/(float)(1L << FORMAT));
        
        // spavim vrednosti v buffer za prikaz
        DLOG_GEN_update();
    }

    
    /* preverim, èe me sluèajno èaka nova prekinitev.
       èe je temu tako, potem je nekaj hudo narobe
       saj je èas izvajanja prekinitve predolg
       vse skupaj se mora zgoditi najmanj 10krat,
       da reèemo da je to res problem
    */
    if (EPwm1Regs.ETFLG.bit.INT == TRUE)
    {
        // povecam stevec, ki steje take dogodke
        interrupt_overflow_counter = interrupt_overflow_counter + 1;
        
        // in ce se je vse skupaj zgodilo 10 krat se ustavim
        // v kolikor uC krmili kakšen resen HW, potem moèno
        // proporoèam lepše "hendlanje" takega dogodka
        // beri:ugasni moènostno stopnjo, ...
        if (interrupt_overflow_counter >= 10)
        {
            asm(" ESTOP0");
        }
    }
    
    // stopam
    TIC_stop();

}   // end of PWM_int

/**************************************************************
* Funckija, ki pripravi vse potrebno za izvajanje
* prekinitvene rutine
**************************************************************/
void PER_int_setup(void)
{

    // Proženje prekinitve 
    EPwm1Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;    //sproži prekinitev na periodo
    EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;         //ob vsakem prvem dogodku
    EPwm1Regs.ETCLR.bit.INT = 1;                //clear possible flag
    EPwm1Regs.ETSEL.bit.INTEN = 1;              //enable interrupt

    // inicializiram data logger
    dlog.trig_value = SWITCH_FREQ - 10;    // specify trigger value
    dlog.slope = Positive;                 // trigger on positive slope
    dlog.prescalar = 1;                    // store every  sample
    dlog.mode = Normal;                    // Normal trigger mode
    dlog.auto_time = 100;                  // number of calls to update function
    dlog.holdoff_time = 100;               // number of calls to update function

    dlog.trig = &ref_counter;
    dlog.iptr1 = &napetost;
    dlog.iptr2 = &tok;


    // registriram prekinitveno rutino
    EALLOW;
    PieVectTable.EPWM1_INT = &PER_int;
    EDIS;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP3;
    PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
    IER |= M_INT3;
    // da mi prekinitev teèe  tudi v real time naèinu
    // (za razhoršèevanje main zanke in BACK_loop zanke)
    SetDBGIER(M_INT3);
}
