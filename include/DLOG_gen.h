/****************************************************************
* FILENAME:     DLOG_gen.h             
* DESCRIPTION:  struct and func declarations for data logger module  
* AUTHOR:       Mitja Nemec
* START DATE:   16.1.2009
* VERSION:      1.1
*
* CHANGES : 
* VERSION   DATE        WHO         DETAIL 
* 1.0       16.1.2009   Mitja Nemec Initial version
* 1.1       23.2.2010   Mitja Nemec Added various trigger modes
*
****************************************************************/
#ifndef     __DLOG_GEN_H__
#define     __DLOG_GEN_H__

// type of element to buffer
#define     DLOG_GEN_TYPE   long
// number of channels
#define     DLOG_GEN_NR     4
// size of each buffer
#define     DLOG_GEN_SIZE   1000

// function prototype
void DLOG_GEN_update(void);

enum MODE { Auto, Normal, Single, Stop};

enum SLOPE { Positive, Negative};

enum STATE { Wait, Ready, Store, Holdoff};

struct DLOG
{
    DLOG_GEN_TYPE   *trig;      // pointer to trigger value
    DLOG_GEN_TYPE   *iptr1;     // Input: First input pointer (Q15)
    #if DLOG_GEN_NR > 1
    DLOG_GEN_TYPE   *iptr2;     // Input: Second input pointer (Q15)
    #endif
    #if DLOG_GEN_NR > 2
    DLOG_GEN_TYPE   *iptr3;     // Input: Third input pointer (Q15)
    #endif
    #if DLOG_GEN_NR > 3
    DLOG_GEN_TYPE   *iptr4;     // Input: Fourth input pointer (Q15)
    #endif
    #if DLOG_GEN_NR > 4
    DLOG_GEN_TYPE   *iptr5;     // Input: Fifth input pointer (Q15)
    #endif
    #if DLOG_GEN_NR > 5
    DLOG_GEN_TYPE   *iptr6;     // Input: Sixth input pointer (Q15)
    #endif
    #if DLOG_GEN_NR > 6
    DLOG_GEN_TYPE   *iptr7;     // Input: Seventh input pointer (Q15)
    #endif
    #if DLOG_GEN_NR > 7
    DLOG_GEN_TYPE   *iptr8;     // Input: Eighth input pointer (Q15)
    #endif

    // common data
    DLOG_GEN_TYPE   trig_value; // Input: Trigger point (Q15)
    int             prescalar;  // Parameter: Data log prescale
    int             skip_cntr;  // Variable:  Data log skip counter
    int             write_ptr;  // Variable:  Graph address pointer
    enum STATE      state;      // state of internal state machine
    enum SLOPE      slope;      // trigger slope 0 - positive, 1 - negative
    long            auto_time;
    long            auto_cnt;   // conter for auto trigger in auto mode
    long            holdoff_time;
    long            holdoff_cnt;
    enum MODE       mode;       // mode selection
};

extern struct DLOG dlog;

#endif // __DLOG_GEN_H__
