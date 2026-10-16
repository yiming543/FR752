/* 
 * File:   gpio.h
 * Author:  yiming
 * Comments: gpio function
 * Revision history: V1.0
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef GPIO_H
#define	GPIO_H

#include <xc.h> // include processor files - each processor file is guarded.  
#include <stdbool.h>

enum eLightState
{
    eOFF = 0,
    eDRL = 1,
    ePOS = 2,
};

extern uint8_t fLightState;

#define HiBeam_ON() HB_EN_SetHigh()
#define HiBeam_OFF() HB_EN_SetLow()

#define PWM_DUTY_0P 0
#define PWM_DUTY_10P (50)
#define PWM_DUTY_100P (500)

//正向輸入
#define TURN_IN TURN_IN_PORT

#define TRUN_ON() DRL_ON
#define TRUN_OFF() DRL_OFF

void POS_ON(void);
void POS_OFF(void);

void DRL_ON(void);
void DRL_OFF(void);
void TURN_IN_DRL_ON(void);
void TURN_IN_DRL_OFF(void);

void HIBEAM_ON(void);
void HIBEAM_OFF(void); 

#endif	/* GPIO_H */

