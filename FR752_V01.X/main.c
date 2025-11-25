/**
  Generated Main Source File

  Company:
    Microchip Technology Inc.

  File Name:
    main.c

  Summary:
    This is the main file generated using PIC10 / PIC12 / PIC16 / PIC18 MCUs

  Description:
    This header file provides implementations for driver APIs for all modules
  selected in the GUI. Generation Information : Product Revision  :  PIC10 /
  PIC12 / PIC16 / PIC18 MCUs - 1.81.8 Device            :  PIC16F1936 Driver
  Version    :  2.00
*/

/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software
   and any derivatives exclusively with Microchip products. It is your
   responsibility to comply with third party license terms applicable to your
   use of third party software (including open source software) that may
   accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/
// 20251125 V01 CS:2EC1
//LIN ==> HiBeam
//LIN ==> DRL 100%
//LIN ==> POS  10%
//TURN_IN ==> TURN_ON


#include "mcc_generated_files/LINDrivers/lin_slave.h"
#include "mcc_generated_files/mcc.h"


extern void DRL_ON(void);
extern void DRL_OFF(void);

#define TURN_IN TURN_IN_PORT
#define TRUN_ON() DRL_ON
#define TRUN_OFF() DRL_OFF

/*
                         Main application
 */
int main(void) {
  // initialize the device
  SYSTEM_Initialize();

  // When using interrupts, you need to set the Global and Peripheral Interrupt
  // Enable bits Use the following macros to:

  // Enable the Global Interrupts
  INTERRUPT_GlobalInterruptEnable();
  // Enable the Peripheral Interrupts
  INTERRUPT_PeripheralInterruptEnable();

  while (1) {
    // Add your application code
    LIN_handler();
    if (TURN_IN) {
      TRUN_ON();
    } else {
      TRUN_OFF();
    }
  }
  return 0;
}
/**
 End of File
*/