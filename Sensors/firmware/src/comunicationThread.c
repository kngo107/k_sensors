//#define r_COMMREPLY 1
//#define r_MOTORRUN 1
//#define r_DEBUG 1
#define r_DEBUG_ms2 1
#define COMMUNICATIONQUEUESIZE 15
#define STARTBYTE 0x80

#include "comunicationThread.h"



COMMUNICATION_DATA communicationData;


// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************


void communication_sendmsg(unsigned char msg, int type)
{
	COMMUNICATION_MESSAGE theMsg;
	theMsg.msg = msg;
	theMsg.type = type;
	if(communicationData.theQueue != 0)
	{
		xQueueSend(communicationData.theQueue, (void*)&(theMsg), 0);
	}
}
void communication_sendmsgISR(unsigned char msg, int type)
{
	COMMUNICATION_MESSAGE theMsg;
	theMsg.msg = msg;
	theMsg.type = type;
	if(communicationData.theQueue != 0)
	{
		xQueueSendFromISR(communicationData.theQueue, (void*)&(theMsg), 0);
	}
}

void communication_send(int left, int right)
{
	communicationData.txField1 = left;
	communicationData.txField2 = right;
}

void communication_sendIntMsg(int left, int right)
{
	COMMUNICATION_MESSAGE theMessage;
	//send message sequence byte
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = STARTBYTE;
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send from rover byte
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = communicationData.TxMsgSeq;	
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send left 1 of 4
	if(left > 9999)
		left = 9999;
	if(right > 9999)
		right = 9999;
	int leftxmit[4];
	int rightxmit[4];
	leftxmit[3] = left % 10;
	left /= 10;
	leftxmit[2] = left % 10;
	left /= 10;
	leftxmit[1] = left % 10;
	left /= 10;
	leftxmit[0] = left % 10;
	rightxmit[3] = right%10;
	right /= 10;
	rightxmit[2] = right%10;
	right /= 10;
	rightxmit[1] = right%10;
	right /= 10;
	rightxmit[0] = right%10;
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(leftxmit[0]);	
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send left 2 of 4
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(leftxmit[1]);
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send left 3 of 4
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(leftxmit[2]);	
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send 4 of 4
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(leftxmit[3]);	
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send right 1 of 4
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(rightxmit[0]);	
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send right 2 of 4
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(rightxmit[1]);	
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send right 3 of 4
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(rightxmit[2]);		
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	//send right 4 of 4
	theMessage.type = communicationData.IntTxMsgSeq;
	theMessage.msg = IntToChar(rightxmit[3]);		
	if(xQueueSend(communicationData.IntQueue, (void*)&(theMessage), 0) == pdTRUE)
	{
		communicationData.IntTxMsgSeq++;
	}
	communicationData.TxMsgSeq++;
	int temp = (int)communicationData.TxMsgSeq;
	if(communicationData.TxMsgSeq == 0x7F)
		communicationData.TxMsgSeq = 0x00;
	PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);	//ENABLE TX INTERRUPT
}

unsigned char communication_getByteISR()
{
	COMMUNICATION_MESSAGE theMessage;
	if(xQueueReceiveFromISR(communicationData.IntQueue, (void*)&(theMessage), 0))
	{
		int i = 0;
		if( theMessage.type == communicationData.IntRxMsgSeq)
		{
			communicationData.IntRxMsgSeq++;
			return theMessage.msg;
		}
		else
		{
			while( theMessage.type != communicationData.IntRxMsgSeq)
			{
				i++;																		
				if(i == COMMUNICATIONQUEUESIZE)
				{
					xQueueReset(communicationData.IntQueue);	//clear queue. we dropped the packet
				}
				xQueueSendToBackFromISR(communicationData.IntQueue, (void*)&(theMessage), 0);	//send to back
				xQueueReceiveFromISR(communicationData.IntQueue, (void*)&(theMessage), 0 );	//get another
			}
			//debugU("COM message loaded\n");
			return theMessage.msg;
		}
	}
}

bool communication_IntQueueEmptyISR()
{
	BaseType_t retval;
	retval = xQueueIsQueueEmptyFromISR(communicationData.IntQueue);
	if(retval == pdTRUE)
		return true;
	else
		return false;
}

void communication_UartTx( char* string )
{
	int i = 0;
	char getChar = ' ';
	while(i < string[i] != 0)
	{
		getChar = string[i];
		DRV_USART1_WriteByte(getChar);
		i++;
	}		
}

char IntToChar(int theInt)
{
	if(theInt == 1)
		return '1';
	else if(theInt == 2)
		return '2';
	else if(theInt == 3)
		return '3';
	else if(theInt == 4)
		return '4';
	else if(theInt == 5)
		return '5';
	else if(theInt == 6)
		return '6';
	else if(theInt == 7)
		return '7';
	else if(theInt == 8)
		return '8';
	else if(theInt == 9)
		return '9';
	else
		return '0';
}

int CharToInt(char theChar)
{
	if(theChar == '1')
		return 1;
	else if(theChar == '2')
		return 2;
	else if(theChar == '3')
		return 3;
	else if(theChar == '4')
		return 4;
	else if(theChar == '5')
		return 5;
	else if(theChar == '6')
		return 6;
	else if(theChar == '7')
		return 7;
	else if(theChar == '8')
		return 8;
	else if(theChar == '9')
		return 9;
	else
		return 0;
}

////####################
//put transmit in interrupt handler. interrupt when queue has room/can send and then pull characters
//the interrupt handly checks its own queue so tha tother threads can use it. 
//make a seperate transmit app file for this... not in system_interrupt.c
//when i want to send a message, enable the interrupt. turn off interrupt when there are no messages in thequeue.
//try to avoid blocks as much as possible for ISRs. also prevents blockings on xmit
void communication_UartTxChar( char theChar )
{
	DRV_USART1_WriteByte(theChar);
}

// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
void COMUNICATIONTHREAD_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
	initDebugU();
    communicationData.state = COMMUNICATION_STATE_INIT;
	communicationData.rxByteCount = 0;
	communicationData.TxMsgSeq = 0x00;
	communicationData.RxMsgSeq = 0x00;
	communicationData.theQueue = xQueueCreate(COMMUNICATIONQUEUESIZE, sizeof(COMMUNICATION_MESSAGE)); //sizeof(communicationData.rxMessage));
	if(communicationData.theQueue == 0)
	{
		crash("E: Comm msgQ");
	}
	communicationData.IntQueue = xQueueCreate(COMMUNICATIONQUEUESIZE, sizeof(COMMUNICATION_MESSAGE)); //sizeof(communicationData.rxMessage));
	if(communicationData.IntQueue == 0)
	{
		crash("E: Int msgQ");
	}
	DRV_USART1_Initialize();
#ifdef r_DEBUG_ms2
	communicationData.msgErr = 0;
#endif
	debugCharInit();
	debugU("Booted\r");
}


void COMUNICATIONTHREAD_Tasks ( void )
{
	while(1)
	{
		//check if queue exists
		if(communicationData.theQueue != 0)	
		{
			//receive a message and store into rxMessage ... block 5 ticks if empty queue
			if(xQueuePeek(communicationData.theQueue, (void*)&(communicationData.rxMessage), portMAX_DELAY ))
			{
				xQueueReceive(communicationData.theQueue, (void*)&(communicationData.rxMessage), portMAX_DELAY);
				//received messageQ message will tell use the state which will be used
				communicationData.state = communicationData.rxMessage.type;
				//communicationData.state = 1;
				
				//run the state according to the message's state
				switch ( communicationData.state )
				{
					/* Application's initial state. */
					case COMMUNICATION_STATE_INIT:	//0
						break;

					case COMMUNICATION_STATE_RECEIVE:	//1
#ifdef r_DEBUG
						debugU("COM rxb: ");
						debugUInt(communicationData.rxMessage.msg);
#endif
						if(communicationData.rxMessage.msg == STARTBYTE)	//received start transmit bit
						{
							if(communicationData.rxByteCount > 0)	//we had part of a previous message...
							{
								debugU("NACK");
#ifdef r_DEBUG_ms2
								communicationData.msgErr++;
								debugU("Message Lost: ");
								debugUInt(communicationData.msgErr);
#endif

								//NACK;
							}
							communicationData.rxBuffer[0] = communicationData.rxMessage.msg;
							communicationData.rxByteCount = 1;	
						}
						else if( communicationData.rxByteCount > 0)	//continuing message
						{
							communicationData.rxBuffer[communicationData.rxByteCount] = communicationData.rxMessage.msg;
							communicationData.rxByteCount++;
							if(communicationData.rxByteCount == 10)
							{
								//check start byte
								if(communicationData.rxBuffer[0] == STARTBYTE)
								{
									//###CHECK seq number
									int i = 0;
									while(communicationData.rxBuffer[1] != communicationData.RxMsgSeq)
									{
										communicationData.RxMsgSeq++;
										debugU("Lost seq num\r");
										i++;
#ifdef r_DEBUG_ms2
										communicationData.msgErr++;
										debugU("Message Lost: ");
										debugUInt(communicationData.msgErr);
#endif
										if(i == 10)
										{
											crash("E: COM too many lost rx seqnum");
										}
									}
									int command = 0;
									command += CharToInt(communicationData.rxBuffer[2]) * 1000;
									command += CharToInt(communicationData.rxBuffer[3]) * 100;
									command += CharToInt(communicationData.rxBuffer[4]) * 10;
									command += CharToInt(communicationData.rxBuffer[5]);
									int duration = 0;
									duration += CharToInt(communicationData.rxBuffer[6]) * 1000;
									duration += CharToInt(communicationData.rxBuffer[7]) * 100;
									duration += CharToInt(communicationData.rxBuffer[8]) * 10;
									duration += CharToInt(communicationData.rxBuffer[9]);
#ifdef r_DEBUG
									debugU("COM com:");
									char msg[12];
									sprintf(msg, "%d", command);
									debugU(msg);
									debugU("	COM dur:");
									sprintf(msg, "%d", duration);
									debugU(msg);
									debugU("\r");
#endif /*r_DEBUG*/
#ifdef r_COMMREPLY
									communication_sendIntMsg(command, duration);
#endif
#ifdef r_MOTORRUN
									debugU("RUNNINGELSE");
									motor_sendmsg(command, duration);
#endif
									communicationData.rxByteCount = 0;
									communicationData.RxMsgSeq++;
									int temp = (int)communicationData.RxMsgSeq;
									if(communicationData.RxMsgSeq == 0x7F)
									{
										communicationData.RxMsgSeq = 0x00;
//										debugU("ROLLOVER\r");
									}
									//ACK
//									debugU("rxseqNum: ");
//									debugUInt(temp);
									PLIB_PORTS_PinToggle(PORTS_ID_0, PORT_CHANNEL_E, 0);
									debugU("\r	ACK\r");
									communicationData.state = 0;
								}//end if check start byte
							}//end if bytecount == 10
						}//end if bytecount > 0
						else	//failed to receive start bit and catch all... nack and reset message
						{
							//NACK
							debugU("NACK");
#ifdef r_DEBUG_ms2
							communicationData.msgErr++;
							debugU("Message Lost: ");
							debugUInt(communicationData.msgErr);
#endif
							communicationData.rxByteCount = 0;	//no start byte and bytecount != 0, so unknown char drop it
						}
						break;

					/* The default state should never be executed. */
					default:
					{
						/* TODO: Handle error in application's state machine. */
						break;
					}
				}//end of case
			}//end of receive message
		}//end of check message queue != 0
		else	//attempt to create the queue again if it doesn't exist
		{
			//something terrible happens normally so have it exit rather than recreate
			crash("E: No Comm Q");
			//run exit interrupt to OS... or forever loop message
			return;
			//communicationData.theQueue = xQueueCreate(10, sizeof(APP_STATES));
		}
	}//end of while(1)
}
 