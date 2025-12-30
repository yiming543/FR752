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
// LIN ==> HiBeam
// LIN ==> DRL 100%
// LIN ==> POS  10%
// TURN_IN ==> TURN_ON

// 20251226 V02 CS:9830
// DRL/POS/LED OFF/HiBeam OK
//修正timeout bug

#include "main.h"

/*
                         Main application
 */
// 啟動UART自動波特率偵測
void AutoBaud_Detect_ON(void) {
  BAUDCONbits.ABDOVF = 0;
  BAUDCONbits.ABDEN = 1;
  BAUDCONbits.WUE = 1;
}

// 設定通信波特率
void AutoBaud_Detect(void) {
  while (1) {
    AutoBaud_Detect_ON();
    while (!BAUDCONbits.ABDOVF) // wait for auto baud to complete
    {
      if (!BAUDCONbits.ABDEN) // if auto baud is disabled, break the loop
      {
        break;
      }
    }

    // 找到正確波特率,沒有超時離開無限迴圈
    if (!BAUDCONbits.ABDOVF) // if auto baud overflowed, try again
    {
      break;
    }
  }
}

// 消除未呼叫警告 假裝有使用這些函數
void Clear_No_Call_Warning(void) {
  uint8_t i = 0;
  if (i == 1) {
    TMR0_Reload();
    TMR0_ReadTimer();
  } else if (i == 2) {
    PIN_MANAGER_IOC();
  } else if (i == 3) {
    EUSART_get_last_status();
    EUSART_is_tx_done();
    EUSART_is_tx_ready();
  } else if (i == 4) {
    TMR2_HasOverflowOccured();
    TMR2_LoadPeriodRegister(0);
    TMR2_WriteTimer(0);
    TMR2_ReadTimer();
    TMR2_StopTimer();
    TMR2_StartTimer();
  } else if (i == 5) {
    PIN_MANAGER_IOC();
  } else if (i == 6) {
    DRL_OFF();
  }
}
int main(void) {
  // initialize the device
  SYSTEM_Initialize();

  // 如果RC7低電位超過500us
  // uint16_t count = 0;
  TMR0_Reload();
  while (1) {
    if (RC7_GetValue() == 1) {
      if(INTCONbits.TMR0IF==1){
        break;
      }
    } else {
      TMR0_Reload();
      INTCONbits.TMR0IF;
    }
  }
  AutoBaud_Detect();

  // When using interrupts, you need to set the Global and Peripheral Interrupt
  // Enable bits Use the following macros to:

  // Enable the Global Interrupts
  INTERRUPT_GlobalInterruptEnable();
  // Enable the Peripheral Interrupts
  INTERRUPT_PeripheralInterruptEnable();

  Clear_No_Call_Warning();
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