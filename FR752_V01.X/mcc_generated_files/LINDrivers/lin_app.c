/**
  LIN Slave Application

  Company:
    Microchip Technology Inc.

  File Name:
    lin_app.c

  Summary:
    LIN Slave Application

  Description:
    This source file provides the interface between the user and
    the LIN drivers.

 */

/*
    (c) 2016 Microchip Technology Inc. and its subsidiaries. You may use this
    software and any derivatives exclusively with Microchip products.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
    WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
    PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION
    WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
    BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
    FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
    ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
    THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.

    MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE
    TERMS.
*/

#include "lin_app.h"
#include "../epwm1.h"
#include "../pin_manager.h"
#include <pic.h>

#define HiBeam_ON() HB_EN_SetHigh()
#define HiBeam_OFF() HB_EN_SetLow()

#define PWM_DUTY_0P 0
#define PWM_DUTY_10P (50)
#define PWM_DUTY_100P (500)

#define MODE_OFF_DRL_24 0x51
#define MODE_POS_24 0x14
#define MODE_POS_24_2 0x12
#define MODE_AUTO_NIGHT_24 0x94
#define MODE_AUTO_NIGHT_24_2 0xAC
#define MODE_LOW_BEAM_24 0x13
// ford ranger 2023
#define MODE_OFF_DRL 0x49
#define MODE_POS 0x0A
#define MODE_AUTO_DAY 0x4C
#define MODE_AUTO_NIGHT 0x8C
#define MODE_LOW_BEAM 0x0B
// 超車
#define mOVER_TAKING 0B10000000 // 0x80 data[1] MASK
// 遠燈
#define mHIBEAM 0B01000000 // 0x40 data[1]
// 左方向
#define mLEFT_TURN_SIGNAL 0B00010000 // 0x10 data[1]
// 右方向
#define mRIGHT_TURN_SIGNAL 0B00100000 // 0x20 data[1]
// 晝行
#define mDRL 0B01000000 // 0x40 data[2]
// 近燈
#define mLOBEAM 0B00001000 // 0x08 data[4]
// 行車
#define mPOS 0B00000011 // 0x03 data[4]

void POS_ON(void) { EPWM1_LoadDutyValue(PWM_DUTY_10P); }
void POS_OFF(void) { EPWM1_LoadDutyValue(PWM_DUTY_0P); }
void DRL_ON(void) { EPWM1_LoadDutyValue(PWM_DUTY_100P); }
void DRL_OFF(void) { EPWM1_LoadDutyValue(PWM_DUTY_0P); }
// void LOBEAM_ON(void) {  }
// void LOBEAM_OFF(void) {  }
void HIBEAM_ON(void) { HiBeam_ON(); }
void HIBEAM_OFF(void) { HiBeam_OFF(); }
// void LEFT_ON(void) {  }
// void LEFT_OFF(void) {  }
// void RIGHT_ON(void) {  }
// void RIGHT_OFF(void) {  }

LampState_t lampState;
// bool fLock = 0; // 1:lock 0:unlock
LampFlags_t lampFlags;

void LIN_Slave_Initialize(void) {

  LIN_init(TABLE_SIZE, scheduleTable, processLIN);
}

void processLIN(void) {
  uint8_t tempRxData[8];
  uint8_t cmd;

  cmd = LIN_getPacket(tempRxData);
  lampState.Byte = 0;
  lampFlags.Byte = 0;

  switch (cmd) {
  case SIGNAL:
    // Lamp mode judgment
    // if ((tempRxData[2] == MODE_OFF_DRL) || (tempRxData[2] ==
    // MODE_OFF_DRL_24)) {
    //   lampState.mode = eOFF_DRL;
    //   lampFlags.fDRL = 1;
    // } else if ((tempRxData[2] == MODE_POS) || (tempRxData[2] == MODE_POS_24)
    // ||
    //            (tempRxData[2] == MODE_POS_24_2)) {
    //   lampState.mode = ePOS;
    //   lampFlags.fPOS = 1;
    // } else if ((tempRxData[2] == MODE_AUTO_NIGHT) ||
    //            (tempRxData[2] == MODE_AUTO_NIGHT_24) ||
    //            (tempRxData[2] == MODE_AUTO_NIGHT_24_2)) {
    //   lampState.mode = eAUTO_LOBEAM_POS;
    //   lampFlags.fLOBEAM = 1;
    //   lampFlags.fPOS = 1;
    // } else if (tempRxData[2] == MODE_AUTO_DAY) {
    //   lampState.mode = eAUTO_DRL;
    //   lampFlags.fDRL = 1;
    // } else if ((tempRxData[2] == MODE_LOW_BEAM) ||
    //            (tempRxData[2] == MODE_LOW_BEAM_24)) {
    //   lampState.mode = eLOBEAM;
    //   lampFlags.fLOBEAM = 1;
    //   lampFlags.fPOS = 1;
    // }

    // 超車
    if ((tempRxData[1] & mOVER_TAKING) == mOVER_TAKING) {
      lampState.OverTaking = 1;
      lampFlags.fHIBEAM = 1;
    }

    // 遠燈
    else if ((tempRxData[1] & mHIBEAM) == mHIBEAM) {
      lampState.HighBeam = 1;
      lampFlags.fHIBEAM = 1;
    }

    // 左方向
    if ((tempRxData[1] & mLEFT_TURN_SIGNAL) == mLEFT_TURN_SIGNAL) {
      lampState.TurnLeft = 1;
      lampFlags.fLEFT = 1;
    }

    // 右方向
    if ((tempRxData[1] & mRIGHT_TURN_SIGNAL) == mRIGHT_TURN_SIGNAL) {
      lampState.TurnRight = 1;
      lampFlags.fRIGHT = 1;
    }

    // 晝行
    if ((tempRxData[2] & mDRL) == mDRL) {
      lampFlags.fDRL = 1;
    }

    // 近燈
    if ((tempRxData[4] & mLOBEAM) == mLOBEAM) {
      lampFlags.fLOBEAM = 1;
    }

    // 行車
    if ((tempRxData[4] & mPOS) == mPOS) {
      lampFlags.fPOS = 1;
    }

    break;

  default:
    lampState.Byte = 0;
    lampFlags.Byte = 0;
    break;
  }

  // Set lamp output
  // DRL
  if (lampFlags.fDRL) {
    DRL_ON();
  }
  else if (lampFlags.fPOS) {
    POS_ON();
  } else {
    POS_OFF();
  }

  // HIBEAM
  if (lampFlags.fHIBEAM) {
    HIBEAM_ON();
  } else {
    HIBEAM_OFF();
  }
}