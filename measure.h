/*
 * measure.h
 *
 *  Created on: 25 mar 2022
 *      Author: michal
 */

#ifndef MEASURE_H_
#define MEASURE_H_

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "mwt_lib.h"

//----------------------------------------------------------------------
//		Private Definitions
//----------------------------------------------------------------------
#define RB_SIZE 	1725 									//size of Raw Buffer
//#define FB_SIZE (RB_SIZE - AVERAGE_OF) 					//size of Filter Buffer
#define AVERAGE_OF 			16								//numbers of samples to mean moving
#define FB_SIZE 			(SAMPLING_100MS - AVERAGE_OF)
#define WINDOW				41								//window, in which we check the extremum, must be ODD
#define WIN_CENTER			((WINDOW - 1)/2) 				//assumption, that center sample in window is potentially extrema
#define DIRECTORY_CHANGES	50								//number of directory changes
#define LAPS_SIZE			DIRECTORY_CHANGES / 2			//number of laps
#define EXTR_SIZE			DIRECTORY_CHANGES * 2			//number of extremes
#define DERIV_SIZE			FB_SIZE
#define	SDIFF				50								//smallest difference between samples in FindExtremum();
#define ACCURACY			400
//----------------------------------------------------------------------
//		Type Definitions
//----------------------------------------------------------------------
typedef enum{
	MEAS_FAIL = 0,
	MEAS_OK
} MeasureStatusTypeDef;

typedef enum{
	EXTR_OLD = 0,
	EXTR_NEW,
	EXTR_REST
} ExtremumStatusTypeDef;

typedef enum{
	DIST_NC = 0, //distance not changed
	DIST_UP 	 //distance upload
}DistanceStatusTypeDef;

typedef struct{
	uint32_t 	buffer[FB_SIZE];
	uint32_t* 	ptr;
}FiltrBuffTypeDef;

typedef struct{
	uint32_t  	buffer[EXTR_SIZE];
	uint32_t	nbuffer[EXTR_SIZE];
	uint32_t*	nptr;
	uint32_t* 	ptr;
	uint8_t 	count;
	uint8_t		relapse;
}ExtremumTypeDef;

typedef struct{
	float		buffer[DERIV_SIZE];
	float*		ptr;
}DerivativeTypeDef;

typedef struct{
	DistanceStatusTypeDef dist_status;
	ExtremumStatusTypeDef extr_status;
	ExtremumTypeDef		  Extr;
	uint32_t next;						//actual length between last extremum and latest filtered sample
	uint32_t prev;						//previous length between last extremum and filtered sample captured 1 second ago
	uint32_t dist_mm;					//actual distance in milimeter
	uint32_t buffer[DIRECTORY_CHANGES];	//store next distances between change directory
	uint32_t dist_base;
	uint8_t	 directory;					//store numbers of changes directory
	uint8_t relapse;
	float meters;
}DistanceTypeDef;

typedef struct{
	uint16_t  n;
	uint32_t raw;
	uint32_t fltrd;
}ReadTypeDef;
//----------------------------------------------------------------------
//		Exported Variables
//----------------------------------------------------------------------

DistanceTypeDef		 	Dist;
FiltrBuffTypeDef		Filtred;
ReadTypeDef				Read;

uint32_t dist_mm;
float dist;
//----------------------------------------------------------------------

//------------------------------------------------------------------------------
//    Exported Functions
//------------------------------------------------------------------------------
//void SumDistances(void);
void CalcDistance(uint16_t n);
uint8_t CalcSegments(uint16_t n);
void FilterSample(uint16_t n);
int16_t ControlEOF(int16_t n);
void ReadFromFile(FILE* fp, uint16_t n);
void FilterSamples_FromFile(void);
void CalcDerivative(void);
int8_t WriteExtrToFile(FILE *fp, uint16_t *n, uint32_t *extr);
int8_t WriteToFile(FILE *fp, uint16_t n,uint32_t *raw, uint32_t *fltr);
int8_t WriteFloatToFile(FILE *fp, float *d);
void DetermineExtremum(FILE* fp, uint16_t n);
void FindExtremum(FILE *fp, uint16_t n);
uint32_t ExtremumFlat(uint32_t *flat);
uint8_t DetectStay(uint32_t *from, uint32_t *to);
void ExtremumRightSide(uint32_t *from, uint32_t *to, uint8_t *r_min, uint8_t *r_max);
void ExtremumLeftSide(uint32_t *from, uint32_t *to, uint8_t *l_min, uint8_t *l_max);
void FindExtremas_FromFile(void);

#endif /* MEASURE_H_ */
