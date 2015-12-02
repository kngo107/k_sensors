#ifndef COMUNICATIONTHREAD_H
#define COMUNICATIONTHREAD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"
#include "communicationThread_public.h"

/*
typedef struct
{
	char seqNum;
	char type;
	int command;
	int duration;
} COMMUNICATION_MESSAGE;
//*/

typedef struct
{
	int type;
	unsigned char msg;
} COMMUNICATION_MESSAGE;


typedef enum
{
	/* Application's state machine's initial state. */
	COMMUNICATION_STATE_INIT=0,
	COMMUNICATION_STATE_RECEIVE=1,
	COMMUNICATION_STATE_RECEIVED=2,
	APP_STATE_RECEIVE=3
} COMMUNICATION_STATES;

typedef struct
{
    COMMUNICATION_STATES state;
	unsigned char rxChar;
	QueueHandle_t theQueue;
	QueueHandle_t IntQueue;
	char TxMsgSeq;
	char RxMsgSeq;
	int IntTxMsgSeq;
	int IntRxMsgSeq;
	int txField1;
	int txField2;
	unsigned char rxBuffer[10];
	int rxByteCount;
#ifdef r_DEBUG_ms2
	int msgErr;
#endif
	COMMUNICATION_MESSAGE rxMessage;
	COMMUNICATION_MESSAGE txMessage;
} COMMUNICATION_DATA;


void communication_UartTx(char* string);
void COMUNICATIONTHREAD_Initialize ( void );
void communication_UartTxChar( char theChar );
char IntToChar(int theInt);
int CharToInt(char theChar);

void COMUNICATIONTHREAD_Tasks( void );


#endif /* _APP_H */