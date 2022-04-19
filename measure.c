/*
 * measure.c
 *
 *  Created on: 25 mar 2022
 *      Author: michal
 */
//------------------------------------------------------------------------------
//    Include
//------------------------------------------------------------------------------
#include <stdlib.h>
#include "measure.h"
#include "mwt_lib.h"
//------------------------------------------------------------------------------
//    Private Types Definitions
//------------------------------------------------------------------------------
MeasureStatusTypeDef  	Meas;
ExtremumTypeDef			Extr_File;
//------------------------------------------------------------------------------
//    Private Defines
//------------------------------------------------------------------------------
#define RB_SIZE 	1725 						//size of Raw Buffer
#define SAMPLING	100							//100ms sampling

//------------------------------------------------------------------------------
//    Private Macros
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//    Global Variables
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//    Private Variable
//------------------------------------------------------------------------------
uint16_t no_of_samples;
uint32_t raw_buffer[RB_SIZE];
uint32_t filtered_buffer[FB_SIZE];

float	 deriv_buffer[FB_SIZE - 1];
float 	 filtered_deriv_value;
float 	 filter_deriv_buffer[FB_SIZE - 1];

char measure_directory[] 				= "/home/michal/Dokumenty/Octave/m03.txt";
char filtered_samples_directory[] 		= "/home/michal/Dokumenty/Octave/f03.txt";
char derivative_directory[] 			= "/home/michal/Dokumenty/Octave/d03.txt";
char filtered_derivative_directory[] 	= "/home/michal/Dokumenty/Octave/x03.txt";
char extremas_directory[]				= "/home/michal/Dokumenty/Octave/e03.txt";
//------------------------------------------------------------------------------
//    Private Functions Prototypes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//    Private Functions
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
//    Functions
//------------------------------------------------------------------------------

void FindExtremum(FILE *fp, uint16_t n){

	uint8_t  status = 0, l_min = 0, l_max = 0, r_min = 0, r_max = 0, tmp = 0;
	uint32_t *start, *mid, *end, *ptr;
	uint32_t extremum = 0;

	start =  Filtred.ptr;
	mid   = (Filtred.ptr - WIN_CENTER); //center
	end	  = (Filtred.ptr - WINDOW + 1); //last

	ExtremumRightSide(start, mid, &r_min, &r_max);
	ExtremumLeftSide(mid, end, &l_min, &l_max);

	//Detect if patient not walking, return status
	status = DetectStay(start, end);

	//ptr to store address of last extremum
	ptr = Dist.Extr.ptr;

	if( ( (l_max + r_max) >= (WINDOW - 2) ) || ( (l_min + r_min) >= (WINDOW - 2) ) ){
		//check if difference is more than, because during random swaying, alghoritm can deteck much more extremas
		if(abs(*ptr - *mid) > 5 * ACCURACY){
			Dist.Extr.ptr++;									//increment pointer to data buffer
			*Dist.Extr.ptr = *mid;					 			//copy extremum to buffer

			Dist.Extr.nptr++;									//increment pointer to number of sample buffer
			*Dist.Extr.nptr = n /*- AVERAGE_OF*/ - WIN_CENTER;		//copy number of sample to buffer

			Dist.Extr.count++;									//iterate counter (number of detected extrema)
			Dist.extr_status = EXTR_NEW;						//set status
			printf("\n----------------\n");
#if(WRITTING_ON)
			WriteExtrToFile(fp, (uint16_t*)Dist.Extr.nptr, Dist.Extr.ptr);
#endif
		}
	}

	else if(status == EXTR_REST){
		extremum = ExtremumFlat(mid);
		//check if difference is more than, because during a rest, alghoritm can deteck much more "flat" extremas
		if(abs(*ptr - extremum) > 5 * ACCURACY){
			Dist.Extr.ptr++;									//increment pointer to data buffer
			*Dist.Extr.ptr = extremum;							//copy extremum to buffer

			Dist.Extr.nptr++;									//increment #endifpointer to number of sample buffer
			*Dist.Extr.nptr = n /*- AVERAGE_OF*/ - WIN_CENTER;		//copy number of sample to buffer

			Dist.Extr.count++;									//iterate counter (number of detected extrema)
			Dist.extr_status = EXTR_NEW;						//set status
			printf("\n----------------\n");
#if(WRITTING_ON)
			WriteExtrToFile(fp, (uint16_t*)Dist.Extr.nptr, Dist.Extr.ptr);
#endif
		}
		/*
		else{
			WriteExtrToFile(fp, (uint16_t*)Dist.Extr.nptr, &raw_meas[n], Filtred.ptr, (uint32_t*)&tmp);
		}
		*/
	}
}

uint32_t ExtremumFlat(uint32_t *flat){

	uint32_t extr = 0;
	extr = *flat;
	return extr;
}

uint8_t DetectStay(uint32_t *from, uint32_t *to){

	uint8_t count = 0;
	uint32_t *first = from - 1;
	uint32_t *second = from;
	uint32_t diff = 0;

	while(first >= to){
		diff = abs(*first - *second);
		if(diff < SDIFF){ 					//change this to something another value
			count++;
		}
		first--, second--;
	}
	if(count >= (WINDOW - 1)){ 				//must be WINDOW - 1
		return EXTR_REST;
	}
	else{
		return EXTR_OLD;
	}
}

void ExtremumRightSide(uint32_t *from, uint32_t *to, uint8_t *r_min, uint8_t *r_max){

	uint32_t *move = from;
	while(move > to){
		if(*move <= *to){
			if(abs(*to - *from) > SDIFF){
				(*r_max)++;
			}
		}
		else if(*move >= *to){
			if(abs(*from - *to) > SDIFF){
				(*r_min)++;
			}
		}
		move--;
	}
}

void ExtremumLeftSide(uint32_t *from, uint32_t *to, uint8_t *l_min, uint8_t *l_max){

	uint32_t *move = from - 1;
	while(move >= to){
		if(*move <= *from){
			if(abs(*from - *to) > SDIFF){
				(*l_max)++;
			}
		}
		else if(*move >= *from){
			if(abs(*to - *from) > SDIFF){
				(*l_min)++;
			}
		}
		move--;
	}
}

/**
  * @brief  Calculate distance
  * @retval none
  */
void CalcDistance(uint16_t n){

	int x1, x2, diff_dist = 0;
	uint32_t* tmp_ptr = Dist.Extr.ptr - 1;

	//set the conditions
	switch(Dist.extr_status){
	case EXTR_OLD:
		x1 = *(Dist.Extr.ptr);									//x1 is actually local extremum
		x2 = (Filtred.buffer[n - AVERAGE_OF]); 					//x2 is last filtered sample
		break;
	case EXTR_NEW:
		x1 = *tmp_ptr;											//x1 is previous extremum
		x2 = *(Dist.Extr.ptr);									//x2 is actual extremum
		break;
	}
	//only in 1st entry to function - initial fill two variables, which will be compared in subsequent iterations n
	if( n == (AVERAGE_OF + SAMPLES_COUNT) ){
		Dist.prev = abs(x2 - x1);								//also dist prev and next are the same
		Dist.next = abs(x2 - x1);
	}

	//in each subsequent loop cycle, n increases and the following condition is executed
	else{
		Dist.prev = Dist.next;									//1 second ago Dist.next id actualy Dist.prev so transcript it
		Dist.next = abs(x2 - x1);								//actual Dist.next is difference between x2 - x1
		diff_dist = abs(Dist.next - Dist.prev);					//diff_dist is auxiliary variable, which store difference between Dist.next and Dist.prev to check, if patient has moved

		if(Dist.extr_status == EXTR_OLD){
			if((Dist.next > Dist.prev) && (diff_dist > 100) ){
				Dist.dist_mm = Dist.dist_base + Dist.next;
			}
		}
		if(Dist.extr_status == EXTR_NEW){
			Dist.dist_base += Dist.next;
			Dist.dist_mm = Dist.dist_base;
			Dist.extr_status = EXTR_OLD;						//set status
		}
		Dist.meters = ((float)Dist.dist_mm / 1000);				//if okay, update meters
	}
}

/**
  * @brief  Determine first extremum, to prove measuring
  * @retval none
  */
void DetermineExtremum(FILE* fp, uint16_t n){
	Dist.Extr.count	 = 1;
	Dist.Extr.ptr 	 = Dist.Extr.buffer;						//assign ptr to buffer[]
	Dist.Extr.nptr 	 = Dist.Extr.nbuffer;						//assign nptr to nbuffer[]
	*(Dist.Extr.ptr) = Filtred.buffer[n - AVERAGE_OF]; 			//enter first filtered sample as first extremum
	*(Dist.Extr.nptr) = n;
#if(WRITTING_ON)
	WriteExtrToFile(fp, (uint16_t*)Dist.Extr.nptr, Dist.Extr.ptr);
#endif
}

int16_t ControlEOF(int16_t n){
	if(n <= Read.n){
		n = Read.n;
		return n;
	}
	else{
		return -1;
	}
}

/**
  * @brief  Read raw measure from .txt
  * @retval none
  */
void ReadFromFile(FILE* fp, uint16_t n){

	char buffer[40] = {0};
	char  n_s[4] = {0}, raw_s[10] = {0}, fltr_s[10] = {0};

	fgets(buffer, 40, fp);
	sscanf(buffer, "%s %s %s", n_s, raw_s, fltr_s);
	Read.n 		= atoi(n_s);
	Read.raw 	= atoi(raw_s);
	Read.fltrd 	= atoi(fltr_s);
}

int8_t WriteExtrToFile(FILE *fp, uint16_t *n, uint32_t *extr){
	int8_t status = 0;
	char buffer[60] = {0};
	sprintf(buffer, "%d %d\n", *n, *extr);

	status = fputs(buffer, fp);
	if(status != -1){
		return 1;
	}
	else{
		return -1;
	}
}

/**
  * @brief  Write datas to .txt
  * @retval 1 OK, -1 NOK
  */
int8_t WriteToFile(FILE *fp, uint16_t n,uint32_t *raw, uint32_t *fltr){

	int8_t status = 0;
	char buffer[40] = {0};

	if(n < AVERAGE_OF){
		sprintf(buffer, "%d %d %d\n", n, *raw, 0);
	}
	else if(n >= AVERAGE_OF){
		sprintf(buffer, "%d %d %d\n", n, *raw, *fltr);
	}
	status = fputs(buffer, fp);
	if(status != -1){
		return 1;
	}
	else{
		return -1;
	}

}

void FilterSample(uint16_t n){

	uint32_t tmp_buffer[AVERAGE_OF] = {0};
	uint32_t filtered_value = 0;

	uint16_t j = AVERAGE_OF;
	uint16_t count;

	for(count = n; count > (n - AVERAGE_OF); count--){
		tmp_buffer[j-1] = raw_meas[count];
		filtered_value += tmp_buffer[j-1];
		j--;
	}

	filtered_value 					= (filtered_value >> 4);
	Filtred.buffer[n - AVERAGE_OF]	= filtered_value;
	Filtred.ptr = &(Filtred.buffer[n - AVERAGE_OF]);
}

//
//NOT USED FUNCTIONS
//@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@22
uint8_t CalcSegments(uint16_t n){

	uint32_t distance = 0;
	int x2 = (Filtred.buffer[n - AVERAGE_OF]);
	int x1 = (Filtred.buffer[n - AVERAGE_OF - SAMPLES_COUNT]);

	distance = abs(x2 - x1);
	if(distance < 200){
		return DIST_NC;
	}
	else{
		dist_mm += distance;
		dist = ( ((float)dist_mm) / 1000 );
		return DIST_UP;
	}
}

/**
  * @brief  Find extremas
  * @retval none
  */
void FindExtremas_FromFile(void){

	uint32_t window[WINDOW] = {0};
	uint32_t *fsample = Filtred.buffer;
	uint32_t *exptr = Extr_File.buffer;
	uint16_t i, j;
	uint8_t  l_min = 0, l_max = 0, r_min = 0, r_max = 0;

	//move the window over all samples
	for(i = 0; i < (FB_SIZE - WIN_CENTER); i++){
		//fullfill window to start analysis
		for(j = 0; j < WINDOW; j++){
			window[j] = *(fsample + j);
		}
		//scan left side from 0 to the middle sample (middle is potential extrema)
		for(j = 0; j < WIN_CENTER; j++){
			//check out the next samples, if they are smaller, potential MAXIMUM, increment variable l_max
			if(window[j] <= window[WIN_CENTER]){
				l_max++;
			}
			//check out the next samples, if they are larger, potential MINIMUM, increment variable l_min
			else if(window[j] >= window[WIN_CENTER]){
				l_min++;
			}
		}
		//scan right side from the middle sample to end of array (middle is potential extrema)
		for(j = (WIN_CENTER + 1); j < WINDOW; j++){
			//check out the next samples, if they are smaller, potential MAXIMUM, increment variable r_max
			if(window[j] <= window[WIN_CENTER]){
				r_max++;
			}
			//check out the next samples, if they are larger, potential MINIMUM, increment variable r_min
			else if(window[j] >= window[WIN_CENTER]){
				r_min++;
			}
		}
		//check numbers of smaller values and largers values than middle sample, if middle sample is the largest or the smallest - EXTREMUM
		if( ( (l_max + r_max) == (WINDOW - 1) ) || ( (l_min + r_min) == (WINDOW - 1) ) ){
			*exptr = window[WIN_CENTER]; //copy sample to buffer
			 exptr++;
		}
		l_max = 0; l_min = 0; r_max = 0; r_min = 0;
		fsample++;
	}
}

/**
  * @brief  Filter samples using moving mean
  * @retval none
  */
void FilterSamples_FromFile(void){

	uint32_t * ptr = raw_buffer; 			//using when filter samples from file
	while(raw_meas[AVERAGE_OF] == 0);		//wait untill first 16 samples will be in buffer
	uint32_t tmp_buffer[AVERAGE_OF];
	uint32_t filtered_value = 0;
	uint16_t i, j;

	FILE *fp;
	fp = fopen(filtered_samples_directory, "w");

	for(i = 0; i < RB_SIZE; i++){

		for(j = 0; j < AVERAGE_OF; j++){
			tmp_buffer[j] = *(ptr + j);
			filtered_value += tmp_buffer[j];
		}

		filtered_value = (filtered_value >> 4);
		Filtred.buffer[i] = filtered_value;
		//WriteToFile(fp, &(Filtred.buffer[i]));
		ptr++;
		filtered_value = 0;
	}
	fclose(fp);
}

/**
  * @brief  Calculate derivative of filtered samples @@@ACTAULLY NOT USED@@@
  * @retval none
  */
void CalcDerivative(void){

	int32_t *x1, *x2, t1, t2;
	uint16_t i;
	float num, denum;

	//FILE *fp;
	//fp = fopen(derivative_directory, "w");

	for(i = 0; i < RB_SIZE; i++){

		if(i+1 != RB_SIZE){
			x1 = (int32_t*)( &(Filtred.buffer[i])   );
			x2 = (int32_t*)( &(Filtred.buffer[i+1]) );
			t1 = SAMPLING * i;
			t2 = SAMPLING * (i+1);
			num = (*x2 - *x1);
			denum = (t2 - t1);

			deriv_buffer[i] = (num / denum);
			//WriteFloatToFile(fp, &(deriv_buffer[i]));
		}
		else{
			break;
		}
	}
}

/**
  * @brief  Write floats to .txt
  * @retval 1 OK, -1 NOK
  */
/*
int8_t WriteFloatToFile(FILE *fp, float *d){

	int8_t status;
	char str[100];

	sprintf(str, "%f\n", *d);
	status = fputs(str, fp);
	if(status != -1){
		return 1;
	}
	else{
		return -1;
	}
}
*/

/**
  * @brief  Filter derivatives, which are calculated from filter sample (fault - double shifting)
  * @retval none
  */
/*
void FilterDerivatives(void){

	float* ptr = deriv_buffer;
	float tmp_buffer[AVERAGE_OF];
	uint16_t i, j;

	FILE *fp;
	fp = fopen(filtered_derivative_directory, "w");

	for(i = 0; i < RB_SIZE; i++){

		for(j = 0; j < AVERAGE_OF; j++){
			tmp_buffer[j] = *(ptr + j);
			filtered_deriv_value += tmp_buffer[j];
		}
		filtered_deriv_val
samples_raw = load ('m03.
		ue = (filtered_deriv_value / AVERAGE_OF);
		filter_deriv_buffer[i] = filtered_deriv_value;
		WriteFloatToFile(fp, &(filter_deriv_buffer[i]));
		ptr++;
		filtered_deriv_value = 0;
	}
	fclose(fp);
}
*/
