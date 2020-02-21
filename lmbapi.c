/*
 * lmbapi.cpp
 *
 *  Created on: Nov 27, 2017
 *      Author: legend
 */
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>
#include <termios.h>
#include <sys/io.h>
#include <fcntl.h>
#include <math.h>
#include "lmbinc.h"
#include "config.h"
#include "iolib.h"


pthread_mutex_t smb_mutex ;	//for SMBus base address 
int32_t flag_smb_mutex = 0 ;	//for SMBus base address 

/*** Variable area ***/
int32_t fBoardReady = 0;



static uint16_t uwCallbackTimer[CB_TOTAL] = {150, 150, 150, 150, 150, 150};

/*** callback thread ***/
pthread_t id_intrCB, id_gpiCB, id_lcmCB, id_hwmCB, id_swrCB, id_psuCB, id_ignCB;
/************************************************/
void __clear_all_mutex(void) 
{

	if ( flag_smb_mutex  ) pthread_mutex_destroy(&smb_mutex);		//SMBus
	flag_smb_mutex = 0;

}
/****/
int32_t __create_mutex(void) 
{
int32_t iRet;
	iRet = pthread_mutex_init(&smb_mutex, NULL);	//initial SMBus access mutex
	if ( iRet != 0 ) {
		__clear_all_mutex();
		return ERR_Error;;
	}
	flag_smb_mutex = 1;
	return ERR_Success;
}

//*******************************************************************************//
//***********  Chapter 3.13     Ignition                    *********************//
//*******************************************************************************//
#if defined(IGN_SUPPORT)
	#include "ign/ignlib.c"
#endif








