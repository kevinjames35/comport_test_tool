//================================================================================================
//		comport_tool.c
//================================================================================================
//Description:
//	this tool is made for testing com port, use two different com port. transmit data from
//one com port to another, and compare the transmit data and receive data show compare result.
//	
//Usage:			
//	./com_util -t [PORT A] -r [PORT B]		-->trans default message from PORT A, receive from PORT B, return compare result
//	./com_util -t [PORT A] -r [PORT B] -s [STRING]	-->trans STRING from PORT A to PORT B, return compare result
//
//Example:
//	./com_util -t /dev/ttyS3 -r /dev/ttyS5				-->/dev/ttyS3 trans default message to /dev/ttyS5
//	./com_util -t /dev/ttyS4 -r /dev/ttyS7 -s customer_message	-->/dev/ttyS4 trans special message to /dev/ttyS7, special message can't include space and need to be under 100 character
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <ctype.h>
#include <sys/ioctl.h>


#define BUFFER_SIZE	1024
int fcom_p1 = -1; 
int fcom_p2 = -1; 

u_int8_t *pWritePtr_p1, *pReadPtr_p1;
u_int8_t *pRxBufferStart_p1, *pRxBufferEnd_p1;
u_int8_t *pWritePtr_p2, *pReadPtr_p2;
u_int8_t *pRxBufferStart_p2, *pRxBufferEnd_p2;
u_int16_t wRxCounter_p1,wRxCounter_p2;
u_int32_t dwBaudrate =0 ;
struct termios old_tios,SerialPortSettings;

pthread_t InQueueID_p1,InQueueID_p2;
pthread_t CheckingThread_p1,CheckingThread_p2;
pthread_mutex_t buf_mut_p1,buf_mut_p2;

u_int8_t RxBuffer_p1[BUFFER_SIZE];

u_int8_t RxBuffer_p2[BUFFER_SIZE];

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
u_int8_t tmpBuffer[60];
int xi,n;
u_int8_t xch;
//printf("incoming\n");
	while(1)
	{
		n=read(fcom_p2, tmpBuffer, 60);
//printf("1receive n=%d,data=%c \n", n,tmpBuffer[0]);
//fflush(stdout);
		if ( n > 0 && n < 60 ) {
//printf("2receive n=%d, \n", n);
			for ( xi=0 ; xi<n ; xi++) {
//printf("%c,",tmpBuffer[xi]);

					//xch = tmpBuffer[xi];
//printf(" %x ", xch);
					pthread_mutex_lock(&buf_mut_p2);				
					//*pWritePtr_p2 = xch;
					*pWritePtr_p2 = tmpBuffer[xi];;
					wRxCounter_p2++;
					if ( pWritePtr_p2 == pRxBufferEnd_p2 ) pWritePtr_p2 = pRxBufferStart_p2;
					else				    pWritePtr_p2++;
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
//printf("%c",xch);			

	//			if ( xch == 0x3a)  {
	//				flag = 1;
//				pReadPtr_p2 = pRxBufferStart_p2;
	//			}
	//			if ( flag == 1){
//				*pReadPtr_p2 = xch;
//					pReadPtr_p2++;
	//			}
			}
//printf("\n");
		}

	}
	return NULL;
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
		if ( wRxCounter_p2 == 0 ) 
		{
			usleep(200); //2ms
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut_p2);
			xch = *pReadPtr_p2;
			*bptr++ = xch;
			wRxCounter_p2--;
			if ( pReadPtr_p2 == pRxBufferEnd_p2) pReadPtr_p2 = pRxBufferStart_p2;
			else				  pReadPtr_p2++;
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
		if ( wRxCounter_p2 == 0 ) 
		{
			sleep(5);
			waitCnt++;
			if ( waitCnt >= 25 )  fEnding = 1;
		}
		else {
			waitCnt = 0;
			pthread_mutex_lock(&buf_mut_p2);
			xch = *pReadPtr_p2;
			if ( xch != 0x3e && xch != 0x0a  && xch != 0x00 ) {
				//maybe ignore all CTRL code 
//printf("---%c",xch);
				*bptr++ = xch;
				rxCnt++;
			}
			wRxCounter_p2--;
			if ( pReadPtr_p2 == pRxBufferEnd_p2) pReadPtr_p2 = pRxBufferStart_p2;
			else				  pReadPtr_p2++;
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
int32_t _OpenPort_p1(char *pComNum)
{
int iResult = 0;
int DTR_flag;
//char pComPath[40];
//printf("%s\n",pComNum);
//sprintf(pComPath,"/dev/ttyS%s",pComNum);
//printf("%s\n",pComPath);
	//fcom_p1 = open("/dev/ttyS4", O_RDWR | O_NDELAY);
fcom_p1 = open(pComNum, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fcom_p1 < 0) return 0;
	//else printf("fcom=%x",fcom_p1);
	//Buffer pointer initial
//	wRxCounter_p1 =0 ;	
//	pWritePtr_p1 = &RxBuffer_p1[0];
//	pReadPtr_p1 = &RxBuffer_p1[0];
//	pRxBufferStart_p1 = &RxBuffer_p1[0];
//	pRxBufferEnd_p1 = &RxBuffer_p1[BUFFER_SIZE-1];
	//hook receiver thread
	//fcom=1;
	//pthread_mutex_init(&buf_mut_p1, NULL);
	

	set_interface_attribs_p1(B115200); 
//pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));

//pthread_create(&InQueueID_p1, (pthread_attr_t*)(0), _PrintIncomeInQueueThread_p1, (void*)(0));

//=================Controll DTR pin===========================
	DTR_flag = TIOCM_DTR;
	ioctl(fcom_p1,TIOCMBIS,&DTR_flag);//set DTR

	//ioctl(fcom_p1,TIOCMBIC,&DTR_flag);//clear DTR 
	iResult = 1;
	
	
	return iResult;
}
/*****************************/
/**** Open COM Port 2    *****/
/*****************************/
int32_t _OpenPort_p2(char *pComNum)
{
int iResult = 0;
//char pComPath[30];
//sprintf(pComPath,"/dev/ttyS%s",pComNum);
//printf("%s\n",pComPath);
	//fcom_p2 = open("/dev/ttyS1", O_RDWR | O_NDELAY);
fcom_p2 = open(pComNum, O_RDWR | O_NOCTTY | O_NDELAY);
	if (fcom_p2 < 0) return 0;
	//else printf("fcom=%x",fcom_p2);
	//Buffer pointer initial
	wRxCounter_p2 =0 ;	
	pWritePtr_p2 = &RxBuffer_p2[0];
	pReadPtr_p2 = &RxBuffer_p2[0];
	pRxBufferStart_p2 = &RxBuffer_p2[0];
	pRxBufferEnd_p2 = &RxBuffer_p2[BUFFER_SIZE-1];
	//hook receiver thread
	//fcom=1;
	pthread_mutex_init(&buf_mut_p2, NULL);
	

	set_interface_attribs_p2(B115200); 
//pthread_create(&InQueueID, (pthread_attr_t*)(0), _IncomeInQueueThread, (void*)(0));

pthread_create(&InQueueID_p2, (pthread_attr_t*)(0), _IncomeInQueueThread_p2, (void*)(0));
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
//char *com_path = "/dev/ttyUSB0";

char rdData[100];
char tdData[100];
int rcvCnt=0;
int count=0;
int data_size=0;
char default_message=1;
	if(argc<2)
	{		
		__printf_usage(argv[0]);
		return 0;
	}
	for(count=0;count<argc;count++)
	{
		if(strcmp("-t",argv[count])==0)
		{
			//printf("select t port%s\n",argv[count+1]);
			iResult = _OpenPort_p1(argv[count+1]);
			if(iResult != 1)
			{
				__printf_usage(argv[0]);
			}
			//else printf("open t port success\n");
		}
		
		if(strcmp("-r",argv[count])==0)
		{
			//printf("select r port%s\n",argv[count+1]);
			iResult = _OpenPort_p2(argv[count+1]);
			if(iResult != 1)
			{
				__printf_usage(argv[0]);
			}
			//else printf("open r port success\n");
		}
		
		if(strcmp("-s",argv[count])==0)
		{
			default_message = 0 ;
			strcpy(tdData,argv[count+1]);
			data_size=sizeof(tdData);
			printf("data size:%d",data_size);
		}
	}


	if(default_message==1)
	{
		strcpy(tdData,"comport test");
		data_size=sizeof(tdData);
		printf("data size:%d",data_size);
	}

	iResult = _SendBufferLength_p1(tdData,12);
	if(iResult == 1)
	{		
		iResult = _ReadBufferLength_p2(rdData,12);
		if(iResult == 1)
		{
			printf("transmit and receive data success\n");
			//printf("%s\n",rdData);
		//	usleep(50);
			if(strncmp(rdData,tdData,1)==0)
			{printf("compare data pass\n");}
			else
			{printf("compare data fail\n");}
		}
		else
		{
			printf("receive data fail\n");
		}

	}
	else
	{
		printf("transmit data fail\n");
	}

		
				
	
	
	close(fcom_p1);
	close(fcom_p2);
	//pthread_cancel(InQueueID_p1);
	pthread_cancel(InQueueID_p2);	
	//pthread_mutex_destroy(&buf_mut_p1);
	pthread_mutex_destroy(&buf_mut_p2);

return 0;

}
