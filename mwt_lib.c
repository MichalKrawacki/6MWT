/*
 * 6mwt_lib.c
 *
 *  Created on: 16 lut 2022
 *      Author: michal
 */
//----------------------------------------------------------------------
//		Includes
//----------------------------------------------------------------------
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h> 		//Contains file controls like O_RDWR
#include <termios.h> 	//Contains POSIX terminal control definitions
#include <errno.h> 		//Error integer and strerror() function
#include <unistd.h> 	//write(), read(), close()
#include <sys/ioctl.h>  //ioctl() check receive buffer on serial port
#include <stdbool.h>
#include <signal.h>

#include "mwt_lib.h"
#include "measure.h"

#include <sys/time.h>


//----------------------------------------------------------------------
//		Private Types Definitions
//----------------------------------------------------------------------
/**
 *
 * @brief Handler for statement of device
 */
typedef struct{
	DeviceStatusTypeDef Status;
	ResStatusTypeDef	Response;
	ResErrorTypeDef		Error;
}HandlerTypeDef;


//----------------------------------------------------------------------
//		Private Definitions
//----------------------------------------------------------------------
#define CMD_STD_SIZE 	2 		//Always must send 2 bytes Type + Length
#define TIMEOUT 		3000

#define MS100			100000	//100ms time (in useconds)
#define MS50			50000	//50ms time (in useconds)

//----------------------------------------------------------------------
//		Private Macros
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//		Global Variables
//----------------------------------------------------------------------
uint8_t  tlv_res_buffer[300];
uint16_t tlv_res_total_len = sizeof(tlv_res_buffer);
uint16_t tlv_res_curr_len = 0;

uint8_t  tlv_req_buffer[300];
uint16_t tlv_req_total_len = sizeof(tlv_req_buffer);
uint16_t tlv_req_curr_len = 0;

uint32_t raw_single_meas;
uint8_t term_signal_set = 0;
//----------------------------------------------------------------------
//		Private Variables
//----------------------------------------------------------------------
struct termios t; 					// Structure of general terminal interface for serial ports
static HandlerTypeDef DWM1001C;
static TLVResTypeDef  TLV_Response;
static TLVReqTypeDef  TLV_Request;

/*
 * 0, 	bytes -> Size of frame to Parse function()
 * 1, 2 bytes -> TLV Request
 * 3, 4 bytes -> TLV Response (TLV2 Type, Length (in bytes))
 * 5, 6 bytes -> TLV Response (TLV3 Type, Length (in bytes))
 */
										// {SIZE of frame, Request, Type_1, Length_1, Type_2, Length_2, Type_3, Length_3}
const uint8_t dwm_pos_set[] 			= {0x02, 0x01, 0x0d};
const uint8_t dwm_pos_get[] 			= {0x04, 0x02, 0x00, 0x41, 0x0d};
const uint8_t dwm_upd_rate_set[] 		= {0x02, 0x03, 0x02};
const uint8_t dwm_upd_rate_get[] 		= {0x04, 0x04, 0x00, 0x45, 0x04};
const uint8_t dwm_cfg_tag_set[] 		= {0x02, 0x05, 0x02};
const uint8_t dwm_cfg_anchor_set[] 		= {0x02, 0x07, 0x01};
const uint8_t dwm_cfg_get[] 			= {0x04, 0x08, 0x00, 0x46, 0x02};
const uint8_t dwm_sleep[] 				= {0x02, 0x0a, 0x00};
uint8_t dwm_loc_get_tag_node[] 			= {0x06, 0x0c, 0x00, 0x41, 0x0d, 0x49, 0x15}; //last value changed from 0x51 to 0x15, because in 1 + 1 setup it's expected TLV length
const uint8_t dwm_loc_get_anch_node[] 	= {0x06, 0x0c, 0x00, 0x41, 0x0d, 0x48, 0xc4}; //202 frame bytes (196 data)
const uint8_t dwm_baddr_set[] 			= {0x02, 0x0f, 0x00};
const uint8_t dwm_baddr_get[] 			= {0x04, 0x10, 0x00, 0x5f, 0x06};
const uint8_t dwm_stnry_cfg_set[] 		= {0x02, 0x11, 0x01};
const uint8_t dwm_stnry_cfg_get[] 		= {0x04, 0x12, 0x00, 0x4a, 0x01};
const uint8_t dwm_factory_reset[] 		= {0x02, 0x13, 0x00};
const uint8_t dwm_reset[] 				= {0x02, 0x14, 0x00};
uint8_t dwm_ver_get[] 					= {0x08, 0x15, 0x00, 0x50, 0x04, 0x51, 0x04, 0x52, 0x04};
const uint8_t dwm_uwb_cfg_set[] 		= {0x02, 0x17, 0x05};
const uint8_t dwm_uwb_cfg_get[] 		= {0x04, 0x18, 0x00, 0x4a, 0x10};
const uint8_t dwm_usr_data_read[] 		= {0x04, 0x19, 0x00, 0x4b, 0x22};
const uint8_t dwm_usr_data_write[] 		= {0x02, 0x1a, 0xff};
const uint8_t dwm_label_read[] 			= {0x04, 0x1c, 0x00, 0x4c, 0x16};
const uint8_t dwm_label_write[] 		= {0x02, 0x1d, 0x10};
const uint8_t dwm_gpio_cfg_output[] 	= {0x02, 0x28, 0x02};
const uint8_t dwm_gpio_cfg_input[] 		= {0x02, 0x29, 0x02};
const uint8_t dwm_gpio_value_set[] 		= {0x02, 0x2a, 0x02};
const uint8_t dwm_gpio_value_get[] 		= {0x04, 0x2b, 0x01, 0x55, 0x01};
const uint8_t dwm_gpio_value_toggle[] 	= {0x02, 0x2c, 0x01};
const uint8_t dwm_panid_set[] 			= {0x02, 0x2e, 0x02};
const uint8_t dwm_panid_get[] 			= {0x04, 0x2f, 0x00, 0x4d, 0x02};
const uint8_t dwm_nodeid_get[] 			= {0x04, 0x30, 0x00, 0x4e, 0x08};
const uint8_t dwm_status_get[] 			= {0x04, 0x32, 0x00, 0x5a, 0x02};
const uint8_t dwm_int_cfg_set[] 		= {0x02, 0x34, 0x02};
const uint8_t dwm_int_cfg_get[] 		= {0x04, 0x35, 0x00, 0x40, 0x02}; //1 specific TLV frame, can interpret this like dummyTLV (0 error_code), and TLV (2 bytes length)
const uint8_t dwm_enc_key_set[] 		= {0x02, 0x3c, 0x10};
const uint8_t dwm_enc_key_clear[] 		= {0x02, 0x3d, 0x00};
const uint8_t dwm_bh_status_get[] 		= {0x04, 0x3a, 0x00, 0x5d, 0x13};
const uint8_t dwm_backhaul_xfer[] 		= {0x02, 0x37, 0x02};

//----------------------------------------------------------------------
//		Private Functions Prototypes
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//		Functions Prototypes
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//		Private Functions
//----------------------------------------------------------------------

//----------------------------------------------------------------------
//		Functions
//----------------------------------------------------------------------

unsigned long GetTickCount(){
 	 struct timeval tv;
  	 gettimeofday(&tv, NULL);
  	 return tv.tv_sec*1000 + tv.tv_usec/1000;
}

unsigned long GetMiliseconds(unsigned long ms){
	struct timeval start, end;
	unsigned long mtime, seconds, useconds;

	gettimeofday(&start, NULL);
	usleep(ms * 1000);
	gettimeofday(&end, NULL);

	seconds = end.tv_sec - start.tv_sec;
	useconds = end.tv_usec - start.tv_usec;

	mtime = ( (seconds*1000) + (useconds/1000.0) + 0.5);
	//printf("%ld\n", mtime);
	return mtime;
}

uint8_t Task(int *fd, TLVResTypeDef* ptr_tlv){

	uint8_t flag = 1;
	int16_t n = 0;
	//unsigned long timeStart = GetTickCount();
	unsigned long timeStart, timeEnd = 0;
	signal(SIGINT, INThandler);						//install Ctrl-C handler to catch a signal and abort loop measuring

	FILE *fpw;
	fpw = fopen("/home/michal/Dokumenty/Octave/m02.txt", "w");

	FILE *fpx;
	fpx = fopen("/home/michal/Dokumenty/Octave/x02.txt", "w");

#if(READING_ON)
	FILE *fpr;
	fpr = fopen("/home/michal/Dokumenty/Octave/m01.txt", "r");
#endif
	while(n < SAMPLING_100MS){

		timeStart = GetTickCount();
		if (term_signal_set > 0){					//flag to check, after CTRL+C term_signal_set = 1
			break;
		}
#if(READING_ON)
		ReadFromFile(fpr, n);
		raw_single_meas = Read.raw;
#elif(!READING_ON)
		//collect 1st 16 samples
		LocalisationGet(fd, ptr_tlv);
#endif
		raw_meas[n] = raw_single_meas;
		raw_single_meas = 0;
		//do 1st filtered sample after collect 16 raw samples
		if(n >= (AVERAGE_OF)){
			FilterSample(n);
			//go inside if only once, at start to save 1st minumum (or maximum)
			if(n == AVERAGE_OF){
				DetermineExtremum(fpx, n);
			}
		}
		//1st condition, because after 1st AVERAGE_OF samples array named filtred_buffer[] store only 1 filtered sample, so must be at least 10 samples (1 second) to calculate distance. SAMPLES_COUNT is fixed "offset".
		if(n == (AVERAGE_OF + SAMPLES_COUNT)){
			CalcDistance(n);
		}
#if(WRITTING_ON)
		WriteToFile(fpw, n, &raw_meas[n], Filtred.ptr);
#endif
		//after another 10 samples, calculate distance (first distance is calculated from 1st raw sample)
		if(n > (AVERAGE_OF + SAMPLES_COUNT) && ( (n % SAMPLES_COUNT) == 0 )){
			CalcDistance(n);
			printf("dist: %.1f m\n", Dist.meters);
		}
		//do this in each iteration after once conditions has been met
		if((n - AVERAGE_OF) >= (WINDOW - 1)){
			FindExtremum(fpx, n);
		}
#if(!READING_ON)
		n++;
		timeEnd = GetTickCount();
		while((GetTickCount() - timeStart) < 100);
#else
		n = ControlEOF(n);
		if(n == -1){
			break;
		}
#endif
	}
#if(WRITTING_ON)
	fclose(fpw);
#elif(READING_ON)
	fclose(fpr);
#endif
	return DWM1001C_OK;
}

uint8_t CollectSamples(int *fd, TLVResTypeDef* ptr_tlv){

	uint16_t i;
	unsigned long time;

	signal(SIGINT, INThandler);						//install Ctrl-C handler to catch a signal and abort loop measuring

	/*uncomennt if want to save raw data to file*/
	FILE *fp;
	fp = fopen("/home/michal/Dokumenty/Octave/m06.txt", "w");

	unsigned long timeStart = GetTickCount();

	for(i = 0; i < SAMPLING_100MS; i++){
		if (term_signal_set > 0){					//flag to check, after CTRL+C term_signal_set = 1
  	      break;
		}
		time = GetTickCount();
		LocalisationGet(fd, ptr_tlv);				//in this function, ParseData() has 50 ms delay

		printf("Raw distance: %d mm\n", raw_single_meas);
		raw_meas[i] = raw_single_meas;
		//SaveToFile(fp);							//uncomennt if want to save raw data to file
		raw_single_meas = 0;						//zeroing variable before next iteration
		while(GetTickCount() - time < 100);
	}

	printf("\nTime: %ld ms\n", GetTickCount() -  timeStart);
	fclose(fp);										//uncomennt if want to save raw data to file

	return 1;
}

void  INThandler(int sig){
	term_signal_set = 1;
    printf("\n***Measure interrupted***\n");
}

uint8_t SaveToFile(FILE *fp){
	int8_t status = 0;

	char buffer[10] = {0};
	sprintf(buffer, "%d\n", raw_single_meas);

	status = fputs(buffer, fp);
	if(status != -1){
		return DWM1001C_OK;
	}
	else{
		return DWM1001C_FAIL;
	}
}

/**
 * @brief  Read distance from device between tag and anchor.
 * @param  fd - FileDescriptor, ptr_tlv - pointer to structure, which store parsed TLV.
 * @retval true - if success, false - not success.
 */
uint8_t LocalisationGet(int *fd, TLVResTypeDef* ptr_tlv){

	raw_single_meas = 0;
	uint8_t status;

	WriteToDevice(fd, (uint8_t*)dwm_loc_get_tag_node);
	status = ParseData(fd, (uint8_t*)dwm_loc_get_tag_node, ptr_tlv);

	if(status == DWM1001C_OK){
		raw_single_meas |= (ptr_tlv -> data_3[3]) << 0;
		raw_single_meas |= (ptr_tlv -> data_3[4]) << 8;
		raw_single_meas |= (ptr_tlv -> data_3[5]) << 16;
		raw_single_meas |= (ptr_tlv -> data_3[6]) << 24;
	}
	else{
		printf("Distance Error");
	}

	ResetBuffer();

return DWM1001C_OK;
}

/**
 * @brief  Reset position of receive buffer.
 * @retval true - success.
 */
uint8_t ResetBuffer(){
	tlv_res_curr_len = 0;

	return 1;
}

/**
 * @brief  Parse raw data from buffer to tlv_res_buff[].
 * @param  fd - FileDescriptor, cmd - command which has info about all respond frames, ptr_tlv - pointer to structure, which store parsed TLV.
 * @retval true - if success, false - not success.
 */
uint8_t ParseData(int *fd, uint8_t* cmd, TLVResTypeDef* ptr_tlv){

	//Extract info about length of whole Frame, and length each TLV,
	uint8_t frame_length = 0, cmd_length = 0;

	cmd_length = *cmd;

	switch(cmd_length){
	case 2:
		frame_length = 3; //1 TLV: 3 bytes
		break;
	case 4:
		frame_length = (3 + 2 + cmd[4]); //2 TLV: 3 bytes(TLV) + 2 bytes(TL) + (V) value from pointer means number of datas
		break;
	case 6:
		frame_length = (3 + 2 + cmd[4] + 2 + cmd[6]); //3 TLV
		break;
	case 8:
		frame_length = (3 + 2 + cmd[4] + 2 + cmd[6] + 2 + cmd[8]); //4 TLV
		break;
	}

	unsigned long start_time = GetTickCount();

	for(;;)
	{
		do{
			usleep(MS50); 							//sleep for 50 ms
			//while((GetTickCount() - delay) < 50); //wait 50 ms for respond
			DWM1001C.Response = ResponseReady(fd, frame_length, &TLV_Response.bytes);

			if(DWM1001C.Response == RESPONSE_NOK || TLV_Response.bytes == 3){
				ReadFromDevice(fd);
				TLV_Response.type_1 	 = tlv_res_buffer[0];
				TLV_Response.length_1 	 = tlv_res_buffer[1];
				TLV_Response.data_1 	 = tlv_res_buffer[2];

				if(TLV_Response.data_1 != TLV_OK) return DWM1001C_FAIL;
				}

			else if(DWM1001C.Response == RESPONSE_OK){
				ReadFromDevice(fd);
				CopyToTLV(cmd);
				(*ptr_tlv) = TLV_Response;
				return DWM1001C_OK;
			}
		}
		while((GetTickCount() - start_time) < TIMEOUT);
		if   ((GetTickCount() - start_time) > TIMEOUT) return DWM1001C_FAIL;
	}
	return DWM1001C_OK;
	}

/**
 * @brief  Copy raw data from tlv_res_buffer[] to dedicated structure.
 * @param  cmd - command.
 * @retval true - if success, false - not success.
 */
uint8_t CopyToTLV(uint8_t *cmd){

		uint8_t no_of_byte = 0, i;
		uint8_t cmd_length = *cmd;

		//parse 1st TLV frame
		TLV_Response.type_1 	 = tlv_res_buffer[0];
		TLV_Response.length_1 	 = tlv_res_buffer[1];
		TLV_Response.data_1 	 = tlv_res_buffer[2];

		//parse 2nd TLV frame
		if(cmd_length >= 4){

			TLV_Response.type_2 	= tlv_res_buffer[3];
			TLV_Response.length_2 	= tlv_res_buffer[4];
			no_of_byte += 5; 						//Store position of 2nd data frame

			for(i = 0; i < TLV_Response.length_2; i++){
			TLV_Response.data_2[i]  = tlv_res_buffer[no_of_byte + i];
			}
			no_of_byte += TLV_Response.length_2; 	//Store position of 3th TLV frame
		}
		//parse 3th TLV frame
		if(cmd_length >= 6){

			TLV_Response.type_3 	= tlv_res_buffer[no_of_byte];
			TLV_Response.length_3	= tlv_res_buffer[no_of_byte + 1];
			no_of_byte += 2;					 	//Store position of 3th data frame

			for(i = 0; i < TLV_Response.length_3; i++){
				TLV_Response.data_3[i] = tlv_res_buffer[no_of_byte + i];
			}
			no_of_byte += TLV_Response.length_3; 	//Store position of 4th TLV frame
		}
		//parse 4th TLV frame
		if(cmd_length >= 8){

			TLV_Response.type_4 	= tlv_res_buffer[no_of_byte];
			TLV_Response.length_4	= tlv_res_buffer[no_of_byte + 1];
			no_of_byte +=2;					 		//Store position of 4th data frame

			for(i = 0; i < TLV_Response.length_4; i++){
				TLV_Response.data_4[i] = tlv_res_buffer[no_of_byte + i];
			}
		}

	return DWM1001C_OK;
}

/**
 * @brief  Check, that number of expected bytes are equal to number of bytes in buffer.
 * @param  fd - FileDescriptor of opened UART port.
 * @retval true - if success, false - not success.
 */
uint8_t ResponseReady(int *fd, uint8_t bytes_expected, uint8_t* bytes_in_buffer){ //Check if all datas are ready to read from buffer, return only statuses.

	int8_t status = 0;
	status = ioctl(*fd, FIONREAD, bytes_in_buffer);	//Generally check actually number of bytes in buffer

	if((*bytes_in_buffer) == bytes_expected){
		return RESPONSE_OK;
	}
	else{
		return RESPONSE_NOK;
	}
}

/**
 * @brief  Read respond from device to tlv_res_buffer[].
 * @param  fd - FileDescriptor of opened UART port.
 * @retval true - if success, false - not success.
 */
uint8_t ReadFromDevice(int *fd){

	int8_t status = 0;
	//uint8_t i = 0; //uncomment to debug

	// printf("***Receive: "); uncomment to debug

	status = read(*fd, &tlv_res_buffer[tlv_res_curr_len], tlv_res_total_len - tlv_res_curr_len);
	if(status > 0){
		tlv_res_curr_len += status;
#ifdef _TEST_PRINT
		for(i = 0; i < tlv_res_curr_len; i++){
			printf("%x ", tlv_res_buffer[i]); //uncomment to debug
		}
#endif
	}
	return 1;
}

/**
 * @brief  Send request to device.
 * @param  fd - FileDescriptor of opened UART port, cmd - 2 bytes hex command.
 * @retval true - if success, false - not success.
 */
uint8_t WriteToDevice(int *fd, uint8_t *cmd){

	int8_t status = 0;
	uint8_t cmd_size = 0;
	uint8_t  i, j;

	if(cmd[2] == 0x00){
		status = write(*fd, &cmd[1], CMD_STD_SIZE);		//standard 2 bytes request
	}
	/*
	 * Test this section, after create functions e.g dwm_cfg_tag_SET() or dwm_..._SET() which writes some datas
	 */
	else{												//sometimes want to transfer some datas - length of this datas are store in structure
		cmd_size = CMD_STD_SIZE + TLV_Request.data_len;
		tlv_req_buffer[0] = cmd[1]; //T
		tlv_req_buffer[1] = cmd[2]; //L

		j = 0, i = 2;
		while(j < TLV_Request.data_len){
			tlv_req_buffer[i] = TLV_Request.data[j];
			i++;
			j++;
		}
		status = write(*fd, tlv_req_buffer, tlv_req_total_len);
	}

	if(status == -1){
		printf("***Sending error, Linux ERRNO=%d\n", errno);
	}

	else{
#ifdef _TEST_PRINT
		printf("\n***Sending status: %d\n", status);
#endif
	}

	return 1;
}

uint8_t TestConnection(int *fd){

	*fd = open("VirtualPort.txt", O_RDWR);
	printf("fd = %d\n", *fd);

	if(*fd != -1){
		printf("Opened VirtalPort.txt\n");
	}
	else{
		printf("Fail Open: VirtalPort.txt\n");
	}

	return 1;
}

/**
 * @brief  Prepare connection with device.
 * @param  fd - FileDescriptor of opened UART port.
 * @retval true - if success, false - not success.
 */
uint8_t Connection(int *fd)
{
	char port[] = "/dev/ttyUSB_";
	int port_number;

	for(port_number = 0; port_number < 50; port_number++){

		printf("\n***Try to open ttyUSB%d\n", port_number);

		port[11] = port_number + '0';
		*fd = open(port, O_RDWR | O_NOCTTY);

		if(*fd >= 0){

			printf("***Input: Success open on ttyUSB%d\n", port_number);
			SetTransmission(fd);


			//WriteToDevice(fd, (uint8_t*)dwm_ver_get, 2);
			//ResponseReady(fd, 21); //After dwm_ver_get device should response in 21 bytes


			return 1; //this must be deleted
			/* !!!!!!! uncomment after finish ResponseReady()
			WriteToDevice(fd, (uint8_t*)dwm_ver_get, 2);
			ReadFromDevice(fd, version_buffer, 21);

			if(version_buffer[0] == 64){
				printf("***6MWT device has been found\n");
				return 1;
			}
			*/
	    }
		else{
			printf("***Input: Error open on ttyUSB%d, Linux ERRNO=%d\n", port_number, errno);
		}
	}
	return 1;
}
/**
 * @brief  UART Port configuration.
 * @param  fd - FileDescriptor of opened UART port.
 * @retval true.
 */
uint8_t SetTransmission(int *fd){

	//Getting the current settings for the port
		tcgetattr(*fd, &t);

	//Setting baudrate (115200)
		cfsetispeed(&t, B115200);
		cfsetospeed(&t, B115200);

	//Modifying c_cflag by bitwise OR and AND. All the flags below, configures standard 8n1 frame
		t.c_cflag &= ~PARENB;
		t.c_cflag &= ~CSTOPB;
		t.c_cflag &= ~CSIZE;
		t.c_cflag &= ~CRTSCTS;

		t.c_cflag |= CS8;
		t.c_cflag |= CLOCAL;

	//Modifying c_lflag by bitwise OR and AND (choose ONE)
		//t.c_lflag |= (ICANON | ECHO | ECHOE); 			//Canonical Input
		t.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); 		//Raw Inpot

	//Modifying c_iflag by bitwise OR and AND
		//t.c_iflag |= ....;
		t.c_iflag &= ~(ICRNL | INLCR | IGNCR | IUCLC); 		//Turn off map CR to NL (and reverse), and ignore CR
		t.c_iflag &= ~(IXON | IXOFF | IXANY); 				//Turn off flow control

	//Modifying c_oflag by bitwise OR and AND
		//t.c_oflag |= ....;
		t.c_oflag &= ~OPOST;

	//Setting the new settings for the port immediately
		tcsetattr(*fd, TCSANOW, &t);

		printf("fd = %d\n", *fd);

	return 1;
}

