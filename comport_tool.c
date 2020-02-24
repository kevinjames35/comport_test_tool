

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <ctype.h>



#define BUFFER_SIZE	1024
int fcom_p1 = -1; 
int fcom_p2 = -1; 

u_int8_t *pWritePtr, *pReadPtr;
u_int8_t *pRxBufferStart, *pRxBufferEnd;
u_int16_t wRxCounter;
u_int32_t dwBaudrate =0 ;
struct termios old_tios,SerialPortSettings;

pthread_t InQueueID_p1,InQueueID_p2;
pthread_t CheckingThread_p1,CheckingThread_p2;
pthread_mutex_t buf_mut_p1,buf_mut_p2;

u_int8_t RxBuffer[BUFFER_SIZE];

int ascii_to_hex(char ch) 
{ 
char ch_tmp; 
int hex_val = -1; 

	ch_tmp = tolower(ch); 

	if ((ch_tmp >= '0') && (ch_tmp <= '9')) { 
		hex_val = ch_tmp - '0'; 
	} 
	else if ((ch_tmp >= 'a') && (ch_tmp <= 'f')) { 
		hex_val = ch_tmp - 'a' + 10; 
	} 

	return hex_val; 
} 
/********************************************************/
int32_t str_to_hex(char *hex_str) 
{
//receive string sample
//: [0x7FFD8B]
int i, len; 
int32_t hex_tmp, hex_val; 
char *bptr;
	bptr = strstr(hex_str, "0x");
	if ( bptr != NULL ) bptr+=2;
	else 	bptr=hex_str;

	len = (int)strlen(bptr); 
	hex_val = 0; 
 	for (i=0; i<len-1;i++) { 
		hex_tmp = ascii_to_hex(bptr[i]); 
		if (hex_tmp == -1) 
		{ return -1; } 

		hex_val = (hex_val) * 16 + hex_tmp; 
	} 
	return hex_val; 
} 
/**********************************************/
float receive_data_analysis(char* buffer)
{
//: [0x800000]-> 0V
//: [0xFFFFFF]-> +10V
	float data;
	data = (( str_to_hex(buffer) - 0x800000 )*(0.000001192));
	//data_buffer = (data_temp - 0x800000)*(0.000001192);
	return data;
	
}
/***********************************************************/
/********************************************************/
/******** for port 1*************************************/
/********************************************************/
int set_interface_attribs_p1 (int speed)
{

tcgetattr(fcom_p1, &SerialPortSettings);
        cfsetospeed (&SerialPortSettings, speed);
        cfsetispeed (&SerialPortSettings, speed);
		// 8N1 Mode 
		SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity   
		SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
		SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size             
		SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8                                 
		
		SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
		
		
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p 
		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                            

		SerialPortSettings.c_oflag |= OPOST;//No Output Processing
		SerialPortSettings.c_oflag |= (ONOCR);//No Output CR
       SerialPortSettings.c_oflag &= ~(ONLCR | OCRNL | ONLRET);//No Output CR
//printf("c flag = %X\n",SerialPortSettings.c_cflag);
//printf("i flag = %X\n",SerialPortSettings.c_iflag);
//printf("o flag = %X\n",SerialPortSettings.c_oflag);
//printf("l flag = %X\n",SerialPortSettings.c_lflag);
        if (tcsetattr (fcom_p1, TCSANOW, &SerialPortSettings) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

/**********************************************/

void* _IncomeInQueueThread_p1(void* object)
{
unsigned char tmpBuffer[60];
int xi,n;
u_int8_t xch;
//printf("incoming\n");
	while(1)
	{

		n=read(fcom_p1, tmpBuffer, 60);
//printf("receive n=%d, \n", n);
//fflush(stdout);
		if ( n > 0 && n < 60 ) {
//printf("receive n=%d, \n", n);
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
//printf("%c", xch);
					pthread_mutex_lock(&buf_mut);				
					*pWritePtr = xch;
					wRxCounter++;
					if ( pWritePtr == pRxBufferEnd ) pWritePtr = pRxBufferStart;
					else				    pWritePtr++;
					pthread_mutex_unlock(&buf_mut);
			}
//printf("\n");
		}
//		else{
//			usleep(5000);
//		}
	}
	return NULL;
}
/**********************************************/

void* _PrintIncomeInQueueThread_p1(void* object)
{
u_int8_t tmpBuffer[60];
int xi,n;
u_int8_t xch;
int flag = 0;

//printf("print incoming\n");
	while(1)
	{
		n=read(fcom_p1, tmpBuffer, 60);
		if ( n > 0 && n < 60 ) {
			for ( xi=0 ; xi<n ; xi++) {
				xch = tmpBuffer[xi];
			
				if ( xch == 0x3a)  {
					flag = 1;
					pReadPtr = pRxBufferStart;
				}
				if ( xch == 0x5D){
					flag = 0;
					*pReadPtr = xch;		
					printf("%8.6f V %s\n",receive_data_analysis(pRxBufferStart),pRxBufferStart);
				}
				if ( flag == 1){
					*pReadPtr = xch;
					pReadPtr++;
				}
			}
//printf("\n");
		}

	}
	return NULL;
}
/******************************************/

int _SendBufferLength_p1(u_int8_t* buffer, int32_t length)
{
u_int8_t *bptr;
	//_ReadBuffer_clear();
	//if ( !fIgnOpened ) return 0;
	if ( length < 1 ) return 0;
	bptr = buffer;
//printf("send CMD=%s\n", bptr);
	write(fcom_p1, bptr, length);
	tcdrain(fcom_p1);
	return 1;
}
/**************/
int _ReadBufferLength_p1(u_int8_t* buffer, int32_t length)
{
int32_t iResult = 0;
int32_t fEnding =0;
int waitCnt, rxCnt;
u_int8_t *bptr, xch;

	if ( !fcom_p1 ) return iResult;
	if ( length < 1 ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wRxCounter == 0 ) 
		{
			usleep(2000); //2ms
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut_p1);
			xch = *pReadPtr;
			*bptr++ = xch;
			wRxCounter--;
			if ( pReadPtr == pRxBufferEnd) pReadPtr = pRxBufferStart;
			else				  pReadPtr++;
			pthread_mutex_unlock(&buf_mut_p1);
			rxCnt++;
			if ( rxCnt == length )  {
				iResult =1;
				fEnding = 1;
			}
		}
	}

	return iResult;
}

/**************/
int _ReadBuffer_p1(int8_t* buffer)
{
int iResult = 0;
int fEnding =0;
int waitCnt, rxCnt;
int8_t *bptr, xch;

	//if ( !fIgnOpened ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wRxCounter == 0 ) 
		{
			sleep(5);
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut_p1);
			xch = *pReadPtr;
			if ( xch != 0x3e && xch != 0x0a  && xch != 0x00 ) {
				//maybe ignore all CTRL code 
//printf("%c",xch);
				*bptr++ = xch;
				rxCnt++;
			}
			wRxCounter--;
			if ( pReadPtr == pRxBufferEnd) pReadPtr = pRxBufferStart;
			else				  pReadPtr++;
			pthread_mutex_unlock(&buf_mut_p1);
			
			if ( (xch == 0x0a) | (xch ==0x3e) )  {

				*bptr = 0x00;
				iResult =1;
				fEnding = 1;
			}
			
		}
	}

	return iResult;
}
/********************************************************/
/******** for port 2*************************************/
/********************************************************/
int set_interface_attribs_p2 (int speed)
{

tcgetattr(fcom_p2, &SerialPortSettings);
        cfsetospeed (&SerialPortSettings, speed);
        cfsetispeed (&SerialPortSettings, speed);
		// 8N1 Mode 
		SerialPortSettings.c_cflag &= ~PARENB;   // Disables the Parity Enable bit(PARENB),So No Parity   
		SerialPortSettings.c_cflag &= ~CSTOPB;   // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit 
		SerialPortSettings.c_cflag &= ~CSIZE;	 // Clears the mask for setting the data size             
		SerialPortSettings.c_cflag |=  CS8;      // Set the data bits = 8                                 
		
		SerialPortSettings.c_cflag &= ~CRTSCTS;       // No Hardware flow Control                         
		SerialPortSettings.c_cflag |= CREAD | CLOCAL; // Enable receiver,Ignore Modem Control lines        
		
		
		SerialPortSettings.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both i/p and o/p 
		SerialPortSettings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode                            

		SerialPortSettings.c_oflag |= OPOST;//No Output Processing
		SerialPortSettings.c_oflag |= (ONOCR);//No Output CR
       SerialPortSettings.c_oflag &= ~(ONLCR | OCRNL | ONLRET);//No Output CR
//printf("c flag = %X\n",SerialPortSettings.c_cflag);
//printf("i flag = %X\n",SerialPortSettings.c_iflag);
//printf("o flag = %X\n",SerialPortSettings.c_oflag);
//printf("l flag = %X\n",SerialPortSettings.c_lflag);
        if (tcsetattr (fcom_p2, TCSANOW, &SerialPortSettings) != 0)
        {
                printf("error %d from tcsetattr", errno);
                return -1;
        }
        return 0;
}

/**********************************************/

void* _IncomeInQueueThread_p2(void* object)
{
unsigned char tmpBuffer[60];
int xi,n;
u_int8_t xch;
//printf("incoming\n");
	while(1)
	{

		n=read(fcom_p2, tmpBuffer, 60);
//printf("receive n=%d, \n", n);
//fflush(stdout);
		if ( n > 0 && n < 60 ) {
//printf("receive n=%d, \n", n);
			for ( xi=0 ; xi<n ; xi++) {
					xch = tmpBuffer[xi];
//printf("%c", xch);
					pthread_mutex_lock(&buf_mut_p2);				
					*pWritePtr = xch;
					wRxCounter++;
					if ( pWritePtr == pRxBufferEnd ) pWritePtr = pRxBufferStart;
					else				    pWritePtr++;
					pthread_mutex_unlock(&buf_mut_p2);
			}
//printf("\n");
		}
//		else{
//			usleep(5000);
//		}
	}
	return NULL;
}
/**********************************************/

void* _PrintIncomeInQueueThread_p2(void* object)
{
u_int8_t tmpBuffer[60];
int xi,n;
u_int8_t xch;
int flag = 0;

//printf("print incoming\n");
	while(1)
	{
		n=read(fcom_p2, tmpBuffer, 60);
		if ( n > 0 && n < 60 ) {
			for ( xi=0 ; xi<n ; xi++) {
				xch = tmpBuffer[xi];
			
				if ( xch == 0x3a)  {
					flag = 1;
					pReadPtr = pRxBufferStart;
				}
				if ( xch == 0x5D){
					flag = 0;
					*pReadPtr = xch;		
					printf("%8.6f V %s\n",receive_data_analysis(pRxBufferStart),pRxBufferStart);
				}
				if ( flag == 1){
					*pReadPtr = xch;
					pReadPtr++;
				}
			}
//printf("\n");
		}

	}
	return NULL;
}
/******************************************/

int _SendBufferLength_p2(u_int8_t* buffer, int32_t length)
{
u_int8_t *bptr;
	//_ReadBuffer_clear();
	//if ( !fIgnOpened ) return 0;
	if ( length < 1 ) return 0;
	bptr = buffer;
//printf("send CMD=%s\n", bptr);
	write(fcom_p2, bptr, length);
	tcdrain(fcom_p2);
	return 1;
}
/**************/
int _ReadBufferLength_p2(u_int8_t* buffer, int32_t length)
{
int32_t iResult = 0;
int32_t fEnding =0;
int waitCnt, rxCnt;
u_int8_t *bptr, xch;

	if ( !fcom_p2 ) return iResult;
	if ( length < 1 ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wRxCounter == 0 ) 
		{
			usleep(2000); //2ms
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut_p2);
			xch = *pReadPtr;
			*bptr++ = xch;
			wRxCounter--;
			if ( pReadPtr == pRxBufferEnd) pReadPtr = pRxBufferStart;
			else				  pReadPtr++;
			pthread_mutex_unlock(&buf_mut_p2);
			rxCnt++;
			if ( rxCnt == length )  {
				iResult =1;
				fEnding = 1;
			}
		}
	}

	return iResult;
}

/**************/
int _ReadBuffer_p2
(int8_t* buffer)
{
int iResult = 0;
int fEnding =0;
int waitCnt, rxCnt;
int8_t *bptr, xch;

	//if ( !fIgnOpened ) return iResult;
	rxCnt=0;
	waitCnt = 0;
	bptr = buffer;
	
	while (!fEnding)
	{
		if ( wRxCounter == 0 ) 
		{
			sleep(5);
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut_p2);
			xch = *pReadPtr;
			if ( xch != 0x3e && xch != 0x0a  && xch != 0x00 ) {
				//maybe ignore all CTRL code 
//printf("%c",xch);
				*bptr++ = xch;
				rxCnt++;
			}
			wRxCounter--;
			if ( pReadPtr == pRxBufferEnd) pReadPtr = pRxBufferStart;
			else				  pReadPtr++;
			pthread_mutex_unlock(&buf_mut_p2);
			
			if ( (xch == 0x0a) | (xch ==0x3e) )  {

				*bptr = 0x00;
				iResult =1;
				fEnding = 1;
			}
			
		}
	}

	return iResult;
}
/*****************************/
/**** Open COM Port 1    *****/
/*****************************/
int32_t _OpenPort_p1(char* pComPath)
{
int iResult = 0;

	//fcom = open("/dev/ttyUSB0", O_RDWR | O_NDELAY);
fcom_p1 = open("/dev/ttyS"+pComPath, O_RDWR | O_NOCTTY);
	if (fcom_p1 < 0) return 0;
	//else printf("fcom=%x",fcom);
	//Buffer pointer initial
	wRxCounter =0 ;	
	pWritePtr = &RxBuffer[0];
	pReadPtr = &RxBuffer[0];
	pRxBufferStart = &RxBuffer[0];
	pRxBufferEnd = &RxBuffer[BUFFER_SIZE-1];
	//hook receiver thread
	//fcom=1;
	pthread_mutex_init(&buf_mut_p1, NULL);
	

	set_interface_attribs(B115200); 
//pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));


	iResult = 1;
	
	
	return iResult;
}
/*****************************/
/**** Open COM Port 2    *****/
/*****************************/
int32_t _OpenPort_p2(char* pComPath)
{
int iResult = 0;

	//fcom = open("/dev/ttyUSB0", O_RDWR | O_NDELAY);
fcom2 = open("/dev/ttyS"+pComPath, O_RDWR | O_NOCTTY);
	if (fcom_p2 < 0) return 0;
	//else printf("fcom=%x",fcom);
	//Buffer pointer initial
	wRxCounter =0 ;	
	pWritePtr = &RxBuffer[0];
	pReadPtr = &RxBuffer[0];
	pRxBufferStart = &RxBuffer[0];
	pRxBufferEnd = &RxBuffer[BUFFER_SIZE-1];
	//hook receiver thread
	//fcom=1;
	pthread_mutex_init(&buf_mut_p2, NULL);
	

	set_interface_attribs(B115200); 
//pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));


	iResult = 1;
	
	
	return iResult;
}
/****************************************************************/
void __printf_usage(char *argv0)
{
		printf("%s -t [PORT A] -r [PORT B]			-->trans default message from PORT A, receive from PORT B, return compare result\n",argv0);
		printf("%s -t [PORT A] -r [PORT B] -s [STRING]	-->trans STRING from PORT A to PORT B, return compare result\n",argv0);
}
/***************************************************************/
int main(int argc, char **argv) 
{
int iResult = 0;
char *com_path = "/dev/ttyUSB0";

char rdData[100];
char tdData[100];
int rcvCnt=0;
int count=0;
char default_message=1;
	if(argc<2)
	{		
		__printf_usage(argv[0]);
		return 0;
	}
	for(count=0;count<argc;count++)
	{
		if(strcmp("-t",count)==0)
		{
			iResult = _OpenPort_p1(argv[count+1]);
			if(iResult != 1)
			{
				__printf_usage(argv[0]);
			}
		}
		
		if(strcmp("-r",count)==0)
		{
			iResult = _OpenPort_p2(argv[count+1]);
			if(iResult != 1)
			{
				__printf_usage(argv[0]);
			}
		}
		
		if(strcmp("-s",count)==0)
		{
			default_message = 0 ;
		}
	}




	pthread_create(&InQueueID_p1, (pthread_attr_t*)(0), _PrintIncomeInQueueThread_p1, (void*)(0));
	pthread_create(&InQueueID_p2, (pthread_attr_t*)(0), _PrintIncomeInQueueThread_p2, (void*)(0));
	iResult = _SendBufferLength_p1("C",1);
	if(iResult == 1)
	{		
		iResult = _ReadBuffer_p2(rdData);
		if(iResult == 1)
		{
			//printf("read success\n");
			//printf("%s\n",rdData);
			//usleep(50000);
		}
		else
		{
			printf("error\n");
		}

	}
	else
	{
		printf("error\n");
	}

		
				
	
	
	close(fcom_p1);
	close(fcom_p2);
	pthread_cancel(InQueueID_p1);
	pthread_cancel(InQueueID_p2);	
	pthread_mutex_destroy(&buf_mut_p1);
	pthread_mutex_destroy(&buf_mut_p2);

return 0;

}