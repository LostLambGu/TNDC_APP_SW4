#ifndef MONET_CAN_H

#define MONET_CAN_H


typedef struct
{

  uint32 StdId;    /*!< Specifies the standard identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x7FF */

  uint32 ExtId;    /*!< Specifies the extended identifier.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32 IDE;      /*!< Specifies the type of identifier for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_Identifier_Type */

  uint32 RTR;      /*!< Specifies the type of frame for the message that will be transmitted.
                          This parameter can be a value of @ref CAN_remote_transmission_request */

  uint32 DLC;      /*!< Specifies the length of the frame that will be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8 Data[8];   /*!< Contains the data to be transmitted.
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

}MonentCanData;

//Message call back function shoud use "OEM_CAN_MSG" defined in pfal.h file. 
//The data structure assicioated with "OEM_CAN_MSG" should be "MonetCANMsgStruct"

typedef enum
{
	CAN_REVICE_MESSAGE =0,  //Receive regular message from CAN bus
	CAN_ERROR_MESSAGE = 1,  //Receive error message from CAN bus	
	CAN_SEND_SUCCESSFUL,    //Send data OK to the CAN bus
	CAN_SEND_FAILED,		//Send data failed to CAN bus
} MonetCANMessageType ;

typedef struct
{
	uint32 msgtype;
	MonentCanData *pData;  //Received CAN message buffer pointer, or Send message data pointer received from at_CANSend function. 
}MonetCANMsgStruct;

void at_CANSend(MonentCanData *pmsg);
void at_CANInit(uint32 baudrate); //baudrate can be 250,000, 500,000, or 1,000,000 or any mode
//void at_CANSetfilter(....); //Please advise how to set the message filter
void at_CANStart(void);
void at_CANStop(void);







#endif
