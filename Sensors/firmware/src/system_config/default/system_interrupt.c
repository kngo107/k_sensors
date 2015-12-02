/*******************************************************************************
 System Interrupts File

  File Name:
    system_int.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <xc.h>
#include <sys/attribs.h>
#include "sensorThread.h"
#include "system_definitions.h"
#include "sensorThread_public.h"


extern volatile bool readyToSendNextByte;
extern volatile bool readyToRxNextbyte;
extern volatile bool startIsDetected;
extern volatile bool addressbyteACKed;
extern volatile bool waitforACKNACK;
extern volatile bool stopIsDetected;
extern volatile bool rollOver;


void IntHandlerDrvTmrInstance0(void)
{
    PLIB_TMR_Counter16BitClear(TMR_ID_2);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);

}

void IntHandlerDrvTmrInstance1(void)
{
    PLIB_TMR_Counter16BitClear(TMR_ID_5);
    rollOver = true;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_5);
    
}


// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************
void IntHandlerDrvI2CInstance0(void) 
{
    static INTERRUPT_STATE interruptState;
    if (PLIB_I2C_StopWasDetected(I2C_ID_1))
    {
        stopIsDetected = true;
        interruptState = I2C_START;
    }
    else if ( (!PLIB_I2C_ReceiverByteAcknowledgeHasCompleted(I2C_ID_1)) &&
            (interruptState == I2C_ACK_NACK_BYTE) )
    {
        
        waitforACKNACK = true;
        interruptState = I2C_TX_RX_DATA_BYTE;
    }
    
    else if ( (PLIB_I2C_ReceivedByteIsAvailable(I2C_ID_1)) &&
            (interruptState == I2C_TX_RX_DATA_BYTE) )
    {
        readyToRxNextbyte = true;
        sendDistancetoQueue(DRV_I2C0_ByteRead());
        interruptState  = I2C_ACK_NACK_BYTE;
    }
    else if ( (PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1)) && (!PLIB_I2C_TransmitterIsBusy(I2C_ID_1)) &&
            (interruptState == I2C_TX_RX_DATA_BYTE) )
    {
        readyToSendNextByte = true;
    }
    else if ( (PLIB_I2C_TransmitterByteHasCompleted(I2C_ID_1)) && (!PLIB_I2C_TransmitterIsBusy(I2C_ID_1)) &&
            (interruptState == I2C_ADDRESS_BYTE) )
    {
        if (PLIB_I2C_TransmitterByteWasAcknowledged(I2C_ID_1))
            addressbyteACKed = true;
        else
            addressbyteACKed = true;
        readyToSendNextByte = true;
        interruptState   = I2C_TX_RX_DATA_BYTE;
    }
    else if ( (interruptState == I2C_START) )
    {
        while(!PLIB_I2C_StartWasDetected(I2C_ID_1));
        startIsDetected = true;
        interruptState = I2C_ADDRESS_BYTE;
    }
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_I2C_1_MASTER);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_I2C_1_ERROR);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_I2C_1_SLAVE);
}
     
void IntHandlerDrvUsartInstance0(void)
{

    /* TODO: Add code to process interrupt here */

    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_2_ERROR);

}

void IntHandlerDrvUsartInstance1(void)
{

    /* TODO: Add code to process interrupt here */
	if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_RECEIVE))
	{
		if(!DRV_USART1_ReceiverBufferIsEmpty()) //grab everyhting in the buffer
		{
			unsigned char msg = DRV_USART1_ReadByte(); // read received byte
			communication_sendmsgISR(msg,1);
		}
	}
	if(PLIB_INT_SourceFlagGet(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT))
	{
		//debugU("INTERRUPT TX\r");
		//check if queue is not empty first...
		if(!communication_IntQueueEmptyISR())
		{
			unsigned char txChar;
			txChar = communication_getByteISR();
			DRV_USART1_WriteByte(txChar);
			//debugU("COM tx: ");
			//debugUInt(txChar);
		}
		else
		{
			PLIB_INT_SourceDisable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);	//disable int due to empty xmit
		}
	}
    /* Clear pending interrupt */
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_RECEIVE);
    PLIB_INT_SourceFlagClear(INT_ID_0, INT_SOURCE_USART_1_ERROR);

}
  
/*******************************************************************************
 End of File
*/

