/*
 * 6mwt_lib.h
 *
 *  Created on: 16 lut 2022
 *      Author: michal
 */

#ifndef MWT_LIB_H_
#define MWT_LIB_H_

//----------------------------------------------------------------------
//		Includes
//----------------------------------------------------------------------
#include "stdbool.h"

//----------------------------------------------------------------------
//		Private Definitions
//----------------------------------------------------------------------
#define SAMPLES_COUNT 10
#define SAMPLING_100MS 3600
#define SAMPLING_200MS 1800

#define WRITTING_ON		0
#define READING_ON		1
//----------------------------------------------------------------------
//		Type Definitions
//----------------------------------------------------------------------
typedef enum{
	DWM1001C_FAIL = 0,
	DWM1001C_OK
} DeviceStatusTypeDef;

typedef enum{
	RESPONSE_NOK = 0,
	RESPONSE_OK,
	RESPONSE_NC,  	//Response not complete
	RESPONSE_ERROR 	//Detected error code
} ResStatusTypeDef;

typedef enum{
	TLV_OK 						= 0,
	TLV_BROKEN_FRAME 			= 1,
	TLV_INTERNAL_ERROR 			= 2,
	TLV_INVALID_PARAMETER 		= 3,
	TLV_BUSY 					= 4,
	TLV_OPERATION_NOT_PERMITTED = 5
} ResErrorTypeDef;

typedef struct{
	uint8_t bytes;
	uint8_t type_1;
	uint8_t length_1;
	uint8_t data_1; //This is only value of error code

	uint8_t type_2;
	uint8_t length_2;
	uint8_t data_2[50];

	uint8_t type_3;
	uint8_t length_3;
	uint8_t data_3[200];

	uint8_t type_4;
	uint8_t length_4;
	uint8_t data_4[10];
}TLVResTypeDef;

typedef struct{
	uint8_t type;
	uint8_t length;
	uint8_t data[256]; // cmd dwm_usr_data_write can send up to 256 bytes data
	uint8_t data_len;
}TLVReqTypeDef;
//----------------------------------------------------------------------
//		Exported Variables
//----------------------------------------------------------------------
uint32_t raw_meas[SAMPLING_100MS];
//----------------------------------------------------------------------
//		Exported Functions
//----------------------------------------------------------------------
uint8_t Task(int *fd, TLVResTypeDef* ptr_tlv);
uint8_t SaveToFile(FILE *fp);
uint8_t CollectSamples(int *fd, TLVResTypeDef* ptr_tlv);
uint8_t ResetBuffer();
uint8_t LocalisationGet(int *fd, TLVResTypeDef* ptr_tlv);
uint8_t ParseData(int *fd, uint8_t* cmd, TLVResTypeDef* ptr_tlv);
uint8_t CopyToTLV(uint8_t *cmd);
uint8_t ResponseReady(int *fd, uint8_t bytes_expected, uint8_t* bytes_in_buffer);
uint8_t WriteToDevice(int *fd, uint8_t *cmd);
uint8_t ReadFromDevice(int *fd);
uint8_t TestConnection(int *fd);
uint8_t Connection(int *fd);
uint8_t SetTransmission(int *fd);

void    INThandler(int);

#endif /* MWT_LIB_H_ */
