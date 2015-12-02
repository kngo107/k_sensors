/* 
 * File:   communication_public.h
 * Author: lucun_000
 *
 * Created on September 27, 2015, 7:49 PM
 */

#ifndef COMMUNICATIONTHREAD_PUBLIC_H
#define	COMMUNICATIONTHREAD_PUBLIC_H	

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "FreeRTOS.h"


#ifdef	__cplusplus
extern "C" {
#endif

void communication_sendmsg(unsigned char msg, int type);
void communication_sendmsgISR(unsigned char msg, int type);
void communication_incrementTxMsgSeq();

void communication_sendIntMsg(int left, int right);
unsigned char communication_getByteISR();
bool communication_IntQueueEmptyISR();

#ifdef	__cplusplus
}
#endif

#endif	/* COMMUNICATION_PUBLIC_H */
