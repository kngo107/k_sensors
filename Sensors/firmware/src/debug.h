/* 
 * File:   debug.h
 * Author: lucun_000
 *
 * //NOTE THAT MAX BAUD RATE FOR DIGIVIEW ACCURATE READS IS 57.6kHz
 * This entire header file can be public.
 * Created on September 17, 2015, 8:01 AM
 */

#ifndef DEBUG_H
#define	DEBUG_H

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

/*
	void debugCharInit();
	void debugChar(int asciiLetter);
//*/

	void initDebugU();
	void debugU(char* debugMessage);
	void crash(char* debugMessage);
	void debugUInt(int number);
	void debugUChar(char debugChar);
	
#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */
