/**
  LIN Slave Application
	
  Company:
    Microchip Technology Inc.

  File Name:
    lin_app.h

  Summary:
    LIN Slave Application

  Description:
    This header file provides the interface between the user and 
    the LIN drivers.

 */

#ifndef LIN_APP_H
#define	LIN_APP_H

#include "lin_slave.h"

typedef union {
  struct {
    _Bool fPOS : 1;
    _Bool fDRL : 1;
    _Bool fLOBEAM : 1;
    _Bool fHIBEAM : 1;
    _Bool fLEFT : 1;
    _Bool fRIGHT : 1;
    _Bool b6 : 1;
    _Bool b7 : 1;
  };
  uint8_t Byte;
} LampFlags_t;

typedef enum {
  eOffDayP=0x09,
  eOffDayN=0x29,
  eParkDayP=0x0A,
  eParkDayN=0x2A,
  eLobeamDayP=0x0B,
  eLobeamDayN=0x2B,
  eAutoDayP=0x0C,
  eAutoDayN=0x4C,
  eOffNightP=0x11,
  eOffNightN=0x31,
  eParkNightP=0x12,
  eParkNightN=0x32,
  eLobeamNightP=0x13,
  eLobeamNightN=0x33,
  eAutoNightP=0x94,
  eAutoNightN=0xB4
}LampControl_t;

typedef enum {
    SIGNAL = 0x01
}lin_cmd_t;

uint8_t SIGNAL_Data[8]; //存放在RAM中，接收資料會存放在此陣列中

const lin_rx_cmd_t scheduleTable[] = { //存放在ROM中，不能改變所指的內容
    //Command, Type, TX/RX Length, Data Address
    {SIGNAL, RECEIVE, 8, SIGNAL_Data }
};
#define TABLE_SIZE  (sizeof(scheduleTable)/sizeof(lin_rx_cmd_t))

void LIN_Slave_Initialize(void);

void processLIN(void);

void POS_ON(void);
void DRL_ON(void); 
void DRL_OFF(void);
void HIBEAM_OFF(void);

#endif	/* LIN_APP_H */

