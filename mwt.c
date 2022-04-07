/*
 ============================================================================
 Name        : 6mwt.c
 Author      : Michal Krawacki
 Version     :
 Copyright   : Your copyright notice
 Description : Hello World in C, Ansi-style
 ============================================================================
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "mwt_lib.h"
#include "measure.h"

#include <sys/ioctl.h>
#include <unistd.h>

int fd = 0;
//----------------------------------------------------------------------

int main(void) {

	TLVResTypeDef tlv;

	Connection(&fd);
	Task(&fd, &tlv);
	//CollectSamples(&fd, &tlv);
	//ReadFromFile();
	//FilterSamples();
	//FindExtremas();

	return EXIT_SUCCESS;
}
