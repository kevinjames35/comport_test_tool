/*
 * ignlib.cpp
 *
 *  Created on: Sep. 03, 2019
 *  Author: legend
 */


#include "ignlib.h"


#define IGN_BUFFER_SIZE	1024
#define MAX_STR_LENGTH	255
#define CMD_GET		"Get "
#define CMD_SET		"Set "
#define qCRLF		"\015\012"

typedef struct DEF_MCU_REPLY {
	int32_t	dwRetCode;
	int8_t 	strCommand[30];
	int8_t	strRetData[30];
}MCU_REPLY;

int fIgnOpened = 0;
int fIgnDevice = -1; 
MCU_REPLY  stuMcuVersion;
int32_t iDID_index = -1;
char strIgnPort[50];

uint8_t *pIgnWritePtr, *pIgnReadPtr;
uint8_t *pIgnRxBufferStart, *pIgnRxBufferEnd;
uint16_t wIgnRxCounter;
uint32_t dwIgnBaudrate =0 ;
struct termios ign_old_tios, ign_new_tios;

pthread_t IgnInQueueID;
//pthread_t CheckingThread;
pthread_mutex_t ign_buf_mut;
pthread_mutex_t ign_port_mut;
uint8_t	IgnRxBuffer[IGN_BUFFER_SIZE];

/*****************/
void* _IgnIncomeInQueueThread(void* object);
int _OpenIgnPort(void);
void _CloseIgnPort(void);
int _IgnSendBufferString(int8_t* buffer);
int _IgnSendBufferLength(int8_t* buffer, int32_t length);
int _IgnReadBuffer_LF(int8_t* buffer);
int _IgnReadBufferLength(uint8_t* buffer, int32_t length);


int32_t __check_return_code(int8_t* pReplyStr, MCU_REPLY* pstuMcuReply);

int32_t __compare_power_state(int8_t* strState);
int32_t __check_did_valid(int8_t* strDeviceID);
int ign_set_interface_attribs (int fd, uint32_t speed, int parity);
void _ign_msdelay(int32_t ms);
void _search_Ign_port(void);

//*******************************************************************************//
//***********  Chapter 3.13      Ignition                   *********************//
//*******************************************************************************//

//***********
//*** 3.13.31
int32_t LMB_IGN_OpenPort(int8_t* pbPortPath)
{
int32_t iResult = ERR_Success;
	if ( !fIgnOpened ) {
		strcpy(strIgnPort, (char*)pbPortPath);
		_OpenIgnPort(); 	
		if ( !fIgnOpened ) iResult = (int32_t)ERR_NotExist; 
	}	
	return iResult;
}
//***********
//*** 3.13.32
int32_t LMB_IGN_ClosePort(void)
{
int32_t iResult = ERR_Success;
	if ( fIgnOpened ) _CloseIgnPort();
	return iResult;
}

//**********
//*** 3.13.2
int32_t Comport_compare(uint8_t port_send, uint8_t port_receive)
{
int32_t iResult = ERR_Success;

	if ( !fIgnOpened ){
		if ( !_OpenIgnPort() ) return (int32_t)ERR_NotOpened;
	} 
		
	//=============>  fill this area for implement
	IGN_CMD_INDEX cIndex;
	//---> check range start
	cIndex = cmdWDT_RETRY_MAX;
	if ( stuStartupScheme.dwMaxFailStartupCnt < _ign_cmdtable[(int32_t)cIndex].dwMinimum ) return (int32_t)ERR_Invalid;
	if ( stuStartupScheme.dwMaxFailStartupCnt > _ign_cmdtable[(int32_t)cIndex].dwMaximum ) return (int32_t)ERR_Invalid;
	cIndex = cmdWATCHDOG_DEFAULT;
	if ( stuStartupScheme.dwStartupWdtTime < _ign_cmdtable[(int32_t)cIndex].dwMinimum ) return (int32_t)ERR_Invalid;
	if ( stuStartupScheme.dwStartupWdtTime > _ign_cmdtable[(int32_t)cIndex].dwMaximum ) return (int32_t)ERR_Invalid;

	//---> check range end
	pthread_mutex_lock(&ign_port_mut);
	int8_t pCmd[60], rdReply[MAX_STR_LENGTH+1];
	MCU_REPLY  stuMcuReply;
	//set WDT_RETRY_MAX
	cIndex = cmdWDT_RETRY_MAX;
	sprintf((char*)pCmd, "%s%s %d%s", (char*)CMD_SET, (char*)_ign_cmdtable[(int32_t)cIndex].strCmd, stuStartupScheme.dwMaxFailStartupCnt, (char*)qCRLF);
	_IgnSendBufferString(pCmd);
	_IgnReadBuffer_LF(rdReply);		
	__check_return_code(rdReply, &stuMcuReply);
	if ( stuMcuReply.dwRetCode != 100 ) iResult= (int32_t)ERR_Error;
	//set WATCHDOG_DEFAULT
	cIndex = cmdWATCHDOG_DEFAULT;
	sprintf((char*)pCmd, "%s%s %d%s", (char*)CMD_SET, (char*)_ign_cmdtable[(int32_t)cIndex].strCmd, stuStartupScheme.dwStartupWdtTime, (char*)qCRLF);
	_IgnSendBufferString(pCmd);
	_IgnReadBuffer_LF(rdReply);		
	__check_return_code(rdReply, &stuMcuReply);
	if ( stuMcuReply.dwRetCode != 100 ) iResult= (int32_t)ERR_Error;
	//set POWERON_DELAY
	cIndex = cmdPOWERON_DELAY;
	sprintf((char*)pCmd, "%s%s %d%s", (char*)CMD_SET, (char*)_ign_cmdtable[(int32_t)cIndex].strCmd, stuStartupScheme.dwKeyOnDelayTime, (char*)qCRLF);
	_IgnSendBufferString(pCmd);
	_IgnReadBuffer_LF(rdReply);		
	__check_return_code(rdReply, &stuMcuReply);
	if ( stuMcuReply.dwRetCode != 100 ) iResult= (int32_t)ERR_Error;
	//set STARTUP_TIMEOUT
	cIndex = cmdSTARTUP_TIMEOUT;
	sprintf((char*)pCmd, "%s%s %d%s", (char*)CMD_SET, (char*)_ign_cmdtable[(int32_t)cIndex].strCmd, stuStartupScheme.dwStartUpTime, (char*)qCRLF);
	_IgnSendBufferString(pCmd);
	_IgnReadBuffer_LF(rdReply);
	__check_return_code(rdReply, &stuMcuReply);
	if ( stuMcuReply.dwRetCode != 100 ) iResult= (int32_t)ERR_Error;
	pthread_mutex_unlock(&ign_port_mut);
	
	return iResult;
	
	return (int32_t)ERR_NotSupport;


}
//***********
//*** 3.13.23
int32_t LMB_IGN_SetPoePower(uint32_t udwPortAssign, uint32_t udwPowerEnable)
{

int32_t iResult = ERR_Success;
uint32_t dwData;

	if ( !fIgnOpened ){
		if ( !_OpenIgnPort() ) return (int32_t)ERR_NotOpened;
	} 

	int8_t pCmd[60], rdReply[MAX_STR_LENGTH+1];


		//=============>  fill this area for implement
		//---> check range start
	if ( udwPortAssign == 0xFFFFFFFF ) udwPortAssign = ign_funcInfo[iDID_index].udwPOEmaxPins;
	if ( udwPortAssign > ign_funcInfo[iDID_index].udwPOEmaxPins   ) return (int32_t)ERR_Invalid; //check port assign is valid
	//---> check range end
	pthread_mutex_lock(&ign_port_mut);	
	//get DIGITAL_POE
	cIndex = cmdDIGITAL_POE;
	sprintf((char*)pCmd, "%s%s%s", (char*)CMD_GET, (char*)_ign_cmdtable[(int32_t)cIndex].strCmd,  (char*)qCRLF);
	_IgnSendBufferString(pCmd);
	_IgnReadBuffer_LF(rdReply);
	__check_return_code(rdReply, &stuMcuReply);

	if ( iResult == ERR_Success ) { 
		dwData &= ~udwPortAssign;
		dwData |= (udwPowerEnable & udwPortAssign);
		//set DIGITAL_POE
		sprintf((char*)pCmd, "%s%s %d%s", (char*)CMD_SET, (char*)_ign_cmdtable[(int32_t)cIndex].strCmd, dwData, (char*)qCRLF);
		_IgnSendBufferString(pCmd);
		_IgnReadBuffer_LF(rdReply);		
		__check_return_code(rdReply, &stuMcuReply);
		if ( stuMcuReply.dwRetCode != 100 ) iResult= (int32_t)ERR_Error;
	}
	pthread_mutex_unlock(&ign_port_mut);


	return iResult;


}






/***************************************************************/
void _ign_msdelay(int32_t ms)
{
struct timeval tvStart, tvEnd;
int fEnding = 0;	
double x1,x2;
	
	gettimeofday(&tvStart, NULL);
	x1 = (double)(tvStart.tv_sec * 1000000 + tvStart.tv_usec) / 1000;
	while (!fEnding) {
		gettimeofday(&tvEnd, NULL);
		x2 = (double)(tvEnd.tv_sec * 1000000 + tvEnd.tv_usec) / 1000;
		if ( (x2-x1) >= ms ) fEnding = 1;
	}	
}
/**********************/
void _check_default_reply(void)
{
int8_t wrCmd[] = "Get VERSION\012";
int8_t rdTemp[MAX_STR_LENGTH+1];

	_IgnSendBufferLength(wrCmd, (int)strlen((char*)wrCmd));
	_IgnReadBuffer_LF(rdTemp);
	__check_return_code(rdTemp,  &stuMcuVersion);
	if ( stuMcuVersion.dwRetCode == 100 ) {
		if ( strcmp((char*)stuMcuVersion.strRetData, (char*)ign_funcInfo[iDID_index].strVersion) > 0 ) {
			ign_funcInfo[iDID_index].fDefaultNoReply = F_DISABLE ;	//version support reply of default command	
		}
	}
}

/****************/
int ign_set_interface_attribs (int fd, uint32_t speed, int parity)
{
        memset (&ign_new_tios, 0, sizeof ign_new_tios);
        if (tcgetattr (fd, &ign_new_tios) != 0)
        {
        //       printf("error from tcgetattr");
                return -1;
        }

        cfsetospeed (&ign_new_tios, speed);
        cfsetispeed (&ign_new_tios, speed);

        ign_new_tios.c_cflag = (ign_new_tios.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
        // disable IGNBRK for mismatched speed tests; otherwise receive break
        // as \000 chars
	//ign_new_tios.c_iflag = 0;  
	ign_new_tios.c_iflag &= ~IGNBRK;         // disable break processing
        ign_new_tios.c_lflag = 0;                // no signaling chars, no echo,
                                        // no canonical processing
        ign_new_tios.c_oflag = 0;                // no remapping, no delays
        ign_new_tios.c_cc[VMIN]  = 0;            // read doesn't block
        ign_new_tios.c_cc[VTIME] = 0;            // 0.5 seconds read timeout

        ign_new_tios.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR); // shut off xon/xoff ctrl
        ign_new_tios.c_cflag |= (CLOCAL | CREAD);	// ignore modem controls,
                                        	// enable reading
        ign_new_tios.c_cflag &= ~(PARENB | PARODD);      // shut off parity
        ign_new_tios.c_cflag |= parity;
        ign_new_tios.c_cflag &= ~CSTOPB;
        ign_new_tios.c_cflag &= ~CRTSCTS;

        if (tcsetattr (fd, TCSANOW, &ign_new_tios) != 0)
        {
                //printf("error from tcsetattr");
                return -1;
        }
        return 0;
}
/**************/
void _ReadBuffer_clear(void)
{

	if ( wIgnRxCounter == 0 ) return ;
	pthread_mutex_lock(&ign_buf_mut);

	wIgnRxCounter=0;
	pIgnReadPtr=pIgnWritePtr;
	pthread_mutex_unlock(&ign_buf_mut);
	return ;
}
/******************************************/
int _IgnSendBufferString(int8_t* buffer)
{
int8_t *bptr;
int32_t length;
	_ReadBuffer_clear();
	if ( !fIgnOpened ) return 0;
	length = strlen((char*)buffer);
	if ( length < 1 ) return 0;
	bptr = buffer;
//printf("send CMD=%s\n", bptr);
	write(fIgnDevice, bptr, length);
	tcdrain(fIgnDevice);
	return 1;
}
/******************************************/
int _IgnSendBufferLength(int8_t* buffer, int32_t length)
{
int8_t *bptr;
	_ReadBuffer_clear();
	if ( !fIgnOpened ) return 0;
	if ( length < 1 ) return 0;
	bptr = buffer;
//printf("send CMD=%s\n", bptr);
	write(fIgnDevice, bptr, length);
	tcdrain(fIgnDevice);
	return 1;
}

/**************/
int _IgnReadBuffer_LF(int8_t* buffer)
{
int iResult = 0;
int fEnding =0;
int waitCnt, rxCnt;
int8_t *bptr, xch;

	if ( !fIgnOpened ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wIgnRxCounter == 0 ) 
		{
			_ign_msdelay(4); //4ms
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&ign_buf_mut);
			xch = *pIgnReadPtr;
			if ( xch != 0x0d && xch != 0x0a  && xch != 0x00 ) {
				//maybe ignore all CTRL code 
				*bptr++ = xch;
				rxCnt++;
			}
			wIgnRxCounter--;
			if ( pIgnReadPtr == pIgnRxBufferEnd) pIgnReadPtr = pIgnRxBufferStart;
			else				  pIgnReadPtr++;
			pthread_mutex_unlock(&ign_buf_mut);
			
			if ( xch == 0x0a || rxCnt >= MAX_STR_LENGTH )  {
				*bptr = 0x00;
				iResult =1;
				fEnding = 1;
			}
			
		}
	}

	return iResult;
}
/**************/
int _IgnReadBufferLength(uint8_t* buffer, int32_t length)
{
int iResult = 0;
int fEnding =0;
int waitCnt, rxCnt;
uint8_t *bptr, xch;

	if ( !fIgnOpened ) return iResult;
	if ( length < 1 ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wIgnRxCounter == 0 ) 
		{
			_ign_msdelay(4); //4ms
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&ign_buf_mut);
			xch = *pIgnReadPtr;
			*bptr++ = xch;
			wIgnRxCounter--;
			if ( pIgnReadPtr == pIgnRxBufferEnd) pIgnReadPtr = pIgnRxBufferStart;
			else				  pIgnReadPtr++;
			pthread_mutex_unlock(&ign_buf_mut);
			rxCnt++;
			if ( rxCnt == length )  {
				iResult =1;
				fEnding = 1;
			}
		}
	}

	return iResult;
}
/*****************/
void _search_Ign_port(void)
{
int xi, iRet;
	for ( xi=0; xi<10; xi++) {
		sprintf(strIgnPort, "/dev/ttyS%d", xi);
		iRet = _OpenIgnPort();
		if ( iRet )  { 
			fIgnOpened = 1;
			xi=100; //exit
		}
		else 	_CloseIgnPort();
	}

}
/*****************************/
/**** Close MCU Port     *****/
/*****************************/
void _CloseIgnPort(void)
{
	pthread_cancel(IgnInQueueID);	
	pthread_mutex_destroy(&ign_buf_mut);
	pthread_mutex_destroy(&ign_port_mut);
	tcsetattr(fIgnDevice, TCSANOW, &ign_old_tios);	//restore setting
	close(fIgnDevice);
	dwIgnBaudrate = 0;
	fIgnDevice=-1;
	iDID_index=-1;
	fIgnOpened = 0;
}
/*****************************/
/**** Open MCU Port      *****/
/*****************************/
int _OpenIgnPort(void)
{
int iResult = 0;
uint32_t baudrate[]={ B57600, B115200,  B38400 };
//uint32_t brValue[] ={  57600,  115200,   38400 };
int8_t wrCmd[] = "Get DEVICE_ID\012";
int8_t wrClr[] = "a\012"; //for clear MCU command buffer when first open
int8_t rdTemp[MAX_STR_LENGTH+1];
uint32_t xi;
int32_t iret;
int port_sel;

fIgnOpened = 1;
for(port_sel=0;port_sel<7;port_sel++)
{
	fIgnDevice = open(*(MCUPORT_PATH_LIST+port_sel), O_RDWR | O_NOCTTY | O_NDELAY);
	
	//fIgnDevice = open((char*)strIgnPort, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fIgnDevice < 0){
		fIgnOpened = 0;
	}
	tcgetattr(fIgnDevice, &ign_old_tios);		//backup setting

	//Buffer pointer initial
	wIgnRxCounter =0 ;	
	pIgnWritePtr = &IgnRxBuffer[0];
	pIgnReadPtr = &IgnRxBuffer[0];
	pIgnRxBufferStart = &IgnRxBuffer[0];
	pIgnRxBufferEnd = &IgnRxBuffer[IGN_BUFFER_SIZE-1];
	//hook receiver thread	
	fIgnOpened=1;
	pthread_mutex_init(&ign_port_mut, NULL);
	pthread_mutex_init(&ign_buf_mut, NULL);
	pthread_create(&IgnInQueueID, (pthread_attr_t*)(0), _IgnIncomeInQueueThread, (void*)(0));
 
	//if ( dwIgnBaudrate == 0 ) { //search 
		for (xi=0 ; xi<(sizeof(baudrate)/4) ; xi++ ) {
			//open LCM Communciaction port 
			ign_set_interface_attribs(fIgnDevice, baudrate[xi], 0); //baudrate 2400 ~ 115200, parity none
			//force clear mcu buffer
			_IgnSendBufferLength(wrClr, (int)strlen((char*)wrClr));
			_IgnReadBuffer_LF(rdTemp);
			
			_IgnSendBufferLength(wrCmd, (int)strlen((char*)wrCmd));
			iret = _IgnReadBuffer_LF(rdTemp);
			if ( iret ) {
				__check_return_code(rdTemp,  &stuMcuVersion);
//printf("stuMcuReply.dwRetCode = %d\n", stuMcuVersion.dwRetCode);
				if ( stuMcuVersion.dwRetCode == 100 ) {
					iDID_index = __check_did_valid(stuMcuVersion.strRetData);
//printf("iDID_index=%d\n", iDID_index);
					if ( iDID_index != -1 ) {
						xi = 1000;
						iResult = 1;		
						_check_default_reply();	
						#if defined(PLATFORM_V6S)								
							_OpenIgnPort_p2();
						#endif
						return iResult;
					}
				}
				else {
					fIgnOpened=0;
				}
			}
			else{
			}

		}
	
	_CloseIgnPort();

}	
	return iResult;
}

/**********************************************/
void* _IgnIncomeInQueueThread(void* object)
{
unsigned char tmpBuffer[512];
int xi,n;
uint8_t xch;

	while(1)
	{
		n=read(fIgnDevice, tmpBuffer, 512);
		if ( n > 0 && n < 512 ) {
//printf("receive n=%d, ", n);
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
//printf("0x%02X ", xch);
					pthread_mutex_lock(&ign_buf_mut);					
					*pIgnWritePtr = xch;
					wIgnRxCounter++;
					if ( pIgnWritePtr == pIgnRxBufferEnd ) pIgnWritePtr = pIgnRxBufferStart;
					else				    pIgnWritePtr++;
					pthread_mutex_unlock(&ign_buf_mut);
			}
//printf("\n");
		}
		else{
			usleep(5000);
		}
	}
	return NULL;
}
/********************/
int32_t __check_return_code(int8_t* pReplyStr, MCU_REPLY* pstuMcuReply)
{

//printf("entry __check_return_code\n");
//printf("MCU reply =%s\n", pReplyStr);
	char *s = strtok((char*)pReplyStr, " "); //分割的判斷字元
	char *put[100]; //分割後放入新的字串陣列
	int s_count=0; //分幾個了

	while(s != NULL) {
	     put[s_count++]=s;  //把分出來的丟進去 結果陣列
	     s = strtok(NULL, " "); //我不知道 這行幹嘛
	     if (s_count >=3 ) break;
	}

	switch ( s_count ) {
	case 3:
		strcpy((char*)pstuMcuReply->strRetData, (char*)put[2]);
	case 2:
		strcpy((char*)pstuMcuReply->strCommand, (char*)put[1]);
	case 1:
		pstuMcuReply->dwRetCode = atoi((char*)put[0]);
		break;
	default: 
		return 0;
	}
	//for(int x=0;x<s_count;x++) //驗收成果
	//      printf("%d %s\n",x,put[x]);
			
	int32_t iResult = atoi((char*)put[0]);
	return iResult;
}


/*********************/
int32_t __compare_power_state(int8_t* strState)
{
int32_t iResult = STAT_UNKNOWN, xi;
int32_t xitem = sizeof(strPowerState) / (int32_t)POWER_SATAE_MAXSTRING;

	for (xi=0 ; xi<xitem ; xi++) {
		if ( strcmp((char*)strState, (char*)strPowerState[xi]) == 0 ) {
			iResult = xi;
		}
	}
	
	return 	iResult;
}
/***********************/
int32_t __check_did_valid(int8_t* strDeviceID)
{
int32_t length = (int32_t)sizeof(ign_funcInfo) /sizeof(IGN_FUNCTIONS);
int32_t iResult=-1,xi;
	for(xi=0 ; xi<length; xi++) {
		if ( strcmp((char*)strDeviceID, (char*)ign_funcInfo[xi].strDeviceID)==0 ) {
			iResult = xi;
			xi = length + 100 ;
		}
	}
	return iResult;
	
}

