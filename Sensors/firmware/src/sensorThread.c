/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    sensorthread.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

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

#include "sensorthread.h"
#include "communicationThread_public.h"
#include "peripheral/oc/plib_oc.h"
// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

/* flags set from interrupt indicating specific operation */
volatile bool readyToSendNextByte = false;
volatile bool readyToRxNextbyte   = false;
volatile bool waitforACKNACK      = false;
volatile bool startIsDetected     = false;
volatile bool addressbyteACKed    = false;
volatile bool stopIsDetected      = false;
volatile bool rollOver            = false;

volatile MASTER_OPERATION master_data_dir;

SENSORTHREAD_DATA sensorthreadData;
int writestate;
int degreeChange;
int degreeStep;
int minDistance;
int minDegree;
int readsPerMove;
int average;
int readstate;
int upperReading;

#define SLAVE_ADDRESS_PIC32         0x62
#define    RegisterMeasure          0x00          
#define    MeasureValue             0x04
#define    ReadReg                  0x10
#define    UpReadReg                0x0f
#define    PI                       3.14159265
#define SAMPLESPERMOVE              5

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback funtions.
*/
void sendDistancetoQueue(uint8_t distance)
{
    if(sensorthreadData.msgQ4 != 0)
	{
		xQueueSendFromISR(sensorthreadData.msgQ4, (void*)&(distance), 0);	
	}
}

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary local functions.
*/


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SENSORTHREAD_Initialize ( void )

  Remarks:
    See prototype in sensorthread.h.
 */

void SENSORTHREAD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    sensorthreadData.state = SENSORTHREAD_STATE_INIT;
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
    sensorthreadData.msgQ4 = xQueueCreate(10, 4);
    
    degreeChange=0;
    average=0;
    minDistance = 20000;
    minDegree = -1;
    readsPerMove = 0;
    upperReading = 0;
    DRV_OC0_Initialize();
    DRV_OC0_Start();
	//DRV_OC0_Stop();
	DRV_TMR0_Initialize();	
	DRV_TMR0_CounterClear();
    DRV_TMR0_Start();
    
    DRV_TMR1_Initialize();	
	DRV_TMR1_CounterClear();
    DRV_TMR1_Start();
	//DRV_TMR0_Stop();
    
}


/******************************************************************************
  Function:
    void SENSORTHREAD_Tasks ( void )

  Remarks:
    See prototype in sensorthread.h.
 */

void SENSORTHREAD_Tasks ( void )
{
    /*
    while (1)
    {
        //Start from 0 degree to 90, 25 mean 1 degree
        if (degreeChange == 0)
        {
            degreeStep = 25;
        }
        if (degreeChange == 2250)
        {
            degreeStep = -25;
        }
        int delay = 600000;
            while (delay >0)
                delay=delay-1;
        PLIB_OC_PulseWidth16BitSet(OC_ID_1, 5900-degreeChange);
        degreeChange += degreeStep;
        debugUInt(degreeChange);
    }*/
    
    while(1)
    {
        /* Check the application's current state. */
        switch ( sensorthreadData.state )
        {
            /* Application's initial state. */
            case SENSORTHREAD_STATE_INIT:
            {
                sensorthreadData.state=SENSORTHREAD_STATE_INIT2;
                master_data_dir = MASTER_WRITE;
               	PLIB_PORTS_DirectionOutputSet(PORTS_ID_0, PORT_CHANNEL_A, 0xC000);
               	PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_15, 1);
                PLIB_PORTS_PinWrite(PORTS_ID_0, PORT_CHANNEL_A, PORTS_BIT_POS_14, 1);
                initDebugU();
                debugU("DEBUGU\r");
                writestate = 0;
                readstate = 0;
                break;
            }
            case SENSORTHREAD_STATE_INIT2:
            {
                if ( DRV_I2C0_MasterBusIdle())
                    sensorthreadData.state=SENSORTHREAD_STATE_START;
                else
                    sensorthreadData.state=SENSORTHREAD_STATE_INIT2;
                break;
            }

            case SENSORTHREAD_STATE_START:
            {
                DRV_I2C0_MasterStart();
                while (!startIsDetected);
                startIsDetected = false;
                sensorthreadData.state = SENSORTHREAD_STATE_SEND_ADDR;
                break;
            }

            case SENSORTHREAD_STATE_SEND_ADDR:
            {
                if (master_data_dir == MASTER_WRITE)
                    DRV_I2C0_ByteWrite(SLAVE_ADDRESS_PIC32 << 1);
                else
                    DRV_I2C0_ByteWrite((SLAVE_ADDRESS_PIC32 << 1) +  0x01);
                
                while (!readyToSendNextByte);
                readyToSendNextByte = false;
                if (addressbyteACKed)
                {
                     if (master_data_dir == MASTER_WRITE)
                         sensorthreadData.state = SENSORTHREAD_STATE_WRITE;
                     else // Calculate THEN READ
                         sensorthreadData.state = SENSORTHREAD_STATE_READ;
                }
                else
                    sensorthreadData.state = SENSORTHREAD_STATE_IDLE;
                break;
            }

            case SENSORTHREAD_STATE_WRITE:
            {
                if (writestate == 0)
                {
                    DRV_I2C0_ByteWrite(RegisterMeasure);
                    while (!readyToSendNextByte);
                    readyToSendNextByte = false;
                    DRV_I2C0_ByteWrite(MeasureValue);
                    while (!readyToSendNextByte);
                    readyToSendNextByte = false;
                    sensorthreadData.state = SENSORTHREAD_STATE_STOP;
                    writestate = 1;
                }
                else if (writestate == 1)
                {
                    DRV_I2C0_ByteWrite(UpReadReg);
                    while (!readyToSendNextByte);
                    readyToSendNextByte = false;
                    sensorthreadData.state = SENSORTHREAD_STATE_STOP;
                    writestate = 2;
                }
                else if (writestate == 2)
                {
                    DRV_I2C0_ByteWrite(ReadReg);
                    while (!readyToSendNextByte);
                    readyToSendNextByte = false;
                    sensorthreadData.state = SENSORTHREAD_STATE_STOP;
                    writestate = 0;
                }
                break;
            }
            case SENSORTHREAD_MOVE_SERVO:
            {
                //Rotate Servo
                //debugUInt(degreeChange/25);
                PLIB_OC_PulseWidth16BitSet(OC_ID_1, 5900-degreeChange);
                degreeChange += degreeStep;
                while (!rollOver);
                rollOver = false;
                sensorthreadData.state = SENSORTHREAD_STATE_STOP;
                break;
            }
            
            case SENSORTHREAD_STATE_READ:
            {
                DRV_I2C0_SetUpByteRead();
                if(sensorthreadData.msgQ4 != 0)	
                {
                    if(xQueueReceive(sensorthreadData.msgQ4, &(sensorthreadData.rxData), portMAX_DELAY ))
                    {
                        //Read from sensor                      
                        int distance = ((int)sensorthreadData.rxData) - 10;
                        if (readstate == 0)
                        {
                            //Upper reading
                            upperReading = distance + 10;
                            readstate = 1;
                        }
                        else if (readstate == 1)
                        {
                            //Lower reading
                            //debugU("Lower\r");
                            readstate = 0;
                        
                            //debugUInt( distance);
                            if (readsPerMove < SAMPLESPERMOVE && distance > 0)
                                average = distance;
                            else
                                average = average;///SAMPLESPERMOVE;
                        }
                        //communication_sendIntMsg((int) sensorthreadData.rxData, 10);
                    }
                }
                while (!readyToRxNextbyte);
                readyToRxNextbyte = false;
                DRV_I2C0_MasterNACKSend();
                while (!waitforACKNACK);
                waitforACKNACK = false;
                if (readsPerMove < SAMPLESPERMOVE)
                {
                    readsPerMove++;
                    sensorthreadData.state = SENSORTHREAD_STATE_STOP;
                }
                else
                {
                    if ((average < minDistance )&&(upperReading == 0))
                    {
                        minDegree = (degreeChange/25);
                        minDistance = average;
                    }

                    if (degreeChange == 0)
                    {     
                        degreeStep = 50;
                        sensorthreadData.state = SENSORTHREAD_CALCULATE;
                    }
                    else if (degreeChange == 2250)
                    {
                        degreeStep = -50;
                        sensorthreadData.state = SENSORTHREAD_CALCULATE;
                    }
                    else 
                        sensorthreadData.state = SENSORTHREAD_MOVE_SERVO;
                    readsPerMove = 0;
                    average = 0;
                }

                //sensorthreadData.state = SENSORTHREAD_STATE_STOP;
                break;
            }
            case SENSORTHREAD_CALCULATE:
            {
                int xcoor = (double)minDistance * (double)cos(minDegree * PI/180.0 );
                xcoor = xcoor - (xcoor/10);
                int ycoor = (double)minDistance * (double)sin(minDegree * PI/180.0 );

                debugU("X:");
                debugUInt((int)xcoor);
                debugU("Y:");
                debugUInt((int)ycoor);
                
                communication_sendIntMsg(xcoor, ycoor);
                
                minDistance = 2000;
                sensorthreadData.state = SENSORTHREAD_MOVE_SERVO;
                break;
            }
            case SENSORTHREAD_STATE_STOP:
            {
                while(!DRV_I2C0_MasterBusIdle());
                DRV_I2C0_MasterStop();
                
                while (!stopIsDetected);
                stopIsDetected = false;
                
                if (master_data_dir == MASTER_WRITE)
                {
                    if (writestate == 2)
                        master_data_dir = MASTER_READ;
                    else if (writestate == 0)
                        master_data_dir = MASTER_READ;
                    
                    sensorthreadData.state = SENSORTHREAD_STATE_INIT2;
                }
                else //master_data_dir == MASTER_READ
                {
                    master_data_dir = MASTER_WRITE;
                    sensorthreadData.state = SENSORTHREAD_STATE_INIT2;
                }
                int delay = 50000;
                while (delay > 0)
                   delay=delay-1;
                break;
            }

            case SENSORTHREAD_STATE_IDLE:
            {
                while(!DRV_I2C0_MasterBusIdle());
                DRV_I2C0_MasterStop();
                
                while (!stopIsDetected);
                stopIsDetected = false;
                sensorthreadData.state = SENSORTHREAD_STATE_INIT2;
                break;
            }

            /* TODO: implement your application state machine.*/

            /* The default state should never be executed. */
            default:
            {
                /* TODO: Handle error in application's state machine. */
                break;
            }
        }
    }
    
}
 

/*******************************************************************************
 End of File
 */
