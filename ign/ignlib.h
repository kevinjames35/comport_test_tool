/*
 * ignlib.h
 *
 *  Created on: Nov 27, 2017
 *      Author: legend
 */

#define F_DISABLE	0	//none
#define F_ENABLE 	1	//supported

#define TDIO_NONE	0	//none
#define TDIO_INOUT	1	//uses DIGITAL_IN & DIGITAL_OUT command
#define TDIO_DIDO	2	//uses DIGITAL_DI & DIGITAL_DO command



const char* MCUPORT_PATH_LIST[]={"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3","/dev/ttyS4","/dev/ttyS5","/dev/ttyS6"}; 
typedef struct DEF_IGN_FUNCTIONS {
	int8_t	 strDeviceID[20];
	int8_t	 strVersion[20];
	int8_t	 fPOE;
	uint32_t udwPOEmaxPins;
	int8_t	 fDItype;
	uint32_t udwDImaxPins;
	int8_t	 fDIinvert;
	int8_t	 fDOtype;
	uint32_t udwDOmaxPins;
	int8_t	 fDOinvert;
	int8_t	 fHeater;
	int8_t	 fPowerAdapter; 
	int8_t	 fDefault;
	int8_t	 fDefaultNoReply;
}IGN_FUNCTIONS;

//==========> fill up here for MCU firmware functions 
IGN_FUNCTIONS ign_funcInfo[]= {
/*DeviceID,     Version,    fPOE,      POEmaxPins, fDItype,    DImaxPins, fDIinvert, fDOtype,   DOmaxPins, fDOinvert, fHeater,   fPowerAdapter, fDefault,  fDefaultNoReply**/
{"LEA-60W930I", "1.21B",    F_DISABLE, 0x00,       TDIO_NONE,  0x00,      F_DISABLE, TDIO_NONE, 0x00,      F_DISABLE, F_DISABLE, F_DISABLE,     F_DISABLE, F_ENABLE},
{"R6S_N",       "0.05B",    F_ENABLE , 0x3FF,      TDIO_DIDO , 0x7F,      F_DISABLE, TDIO_DIDO, 0x7F,      F_ENABLE,  F_ENABLE,  F_DISABLE,     F_ENABLE,  F_ENABLE},
{"V6S_N", 	"0.04B_PI", F_ENABLE,  0x3FF,      TDIO_DIDO,  0x3F,      F_DISABLE, TDIO_DIDO, 0x3F,      F_DISABLE, F_DISABLE, F_DISABLE,     F_ENABLE,  F_ENABLE},
};

/************** Ignition *************/
typedef struct DEF_IGN_COMMAND {
	int8_t		strCmd[33];
	int32_t		dwMinimum;
	int32_t		dwMaximum;	
	int32_t		dwDefault;
}IGN_COMMAND;

/*** Ignition define  ***/
typedef enum DEF_IGN_CMD_INDEX {
	cmdPOWERON_DELAY = 0,
	cmdSTARTUP_TIMEOUT,
	cmdSHUTDOWN_DELAY,
	cmdSHUTDOWN_TIMEOUT,
	cmdLOWPOWER_DELAY,
	cmdPOWER_STATE,
	cmdWATCHDOG_TIMER,
	cmdWATCHDOG_DEFAULT,
	cmdCOUNTDOWN_TIMER,
	cmdINPUT_VOLTAGE_MIN,
	cmdINPUT_VOLTAGE_MAX,
	cmdSTARTUP_VOLTAGE,
	cmdFAIL_COUNT,
	cmdFAIL_RETRY,
	cmdWDT_RETRY_COUNT,
	cmdWDT_RETRY_MAX,
	cmdPOWEROFF_VOLTAGE,
	cmdPOWEROFF_DELAY,
	cmdPOWEROFF_TIMER,
	cmdINPUT_VOLTAGE,
	cmdDEVICE_ID,
	cmdVERSION,
	cmdSW_SHUTDOWN,
	cmdDELAY_ON,
	cmdDIGITAL_OUT,
	cmdDIGITAL_DO,
	cmdDIGITAL_POE,
	cmdDIGITAL_IN,
	cmdDIGITAL_DI,
	cmdLOWPOWER_DISABLE,
	cmdWAKEUP_ENABLE,
	cmdWATCHDOG_ENABLE,
	cmdGPIO_ENABLE,
	cmdLOWPOWER_ENABLE,
	cmdIGNITION,
	cmdGPIO,
	cmdINPUT_TEMP_MIN,
	cmdINPUT_TEMP_MAX,
	cmdINPUT_TEMP
}IGN_CMD_INDEX;

IGN_COMMAND _ign_cmdtable[] = {
/* command,              Minimum, Maximum, default  */
#if defined(PLATFORM_LVC2001)
{"POWERON_DELAY"	, 4	, 604800 , 4 	},
{"STARTUP_TIMEOUT"	, 10	, 604800 , 60 	},
{"SHUTDOWN_DELAY" 	, 0	, 604800 , 4 	},
{"SHUTDOWN_TIMEOUT" 	, 10	, 604800 , 60 	},
{"LOWPOWER_DELAY" 	, 0	, 604800 , 30	},
{"POWER_STATE" 		, -1	, -1  	 , -1 	},
{"WATCHDOG_TIMER" 	, -1	, 604800 , -1 	},
{"WATCHDOG_DEFAULT" 	, 0	, 604800 , 300 	},
{"COUNTDOWN_TIMER" 	, -1	, 604800 , -1 	},
{"INPUT_VOLTAGE_MIN" 	, 0	, 500000 , 8500 },
{"INPUT_VOLTAGE_MAX" 	, 0	, 500000 , 38000},
{"STARTUP_VOLTAGE" 	, 0	, 500000 , 0 	},
{"FAIL_COUNT" 		, 0	, 2000   , 0 	},
{"FAIL_RETRY" 		, 0	, 2000   , 0 	},
{"WDT_RETRY_COUNT" 	, 0	, 1024   , 0 	},
{"WDT_RETRY_MAX" 	, 0	, 1024   , 3 	},
{"POWEROFF_VOLTAGE" 	, 0	, 500000 , 0 	},
{"POWEROFF_DELAY" 	, 0	, 604800 , 10 	},
{"POWEROFF_TIMER" 	, -1	, -1     , -1	},
{"INPUT_VOLTAGE" 	, -1	, -1     , -1 	},
{"DEVICE_ID" 		, -1	, -1     , -1 	},
{"VERSION" 		, -1	, -1     , -1 	},
{"SW_SHUTDOWN" 		, -1	, -1     , -1 	},
{"DELAY_ON" 		, -1	, 604800 , -1 	},
{"DIGITAL_OUT" 		, 0	, 65535  , 0 	},
{"DIGITAL_DO" 		, 0	, 65535  , 0 	},
{"DIGITAL_POE" 		, 0	, 65535  , 65535},
{"DIGITAL_IN" 		, -1	, -1     , -1 	},
{"DIGITAL_DI" 		, -1	, -1     , -1 	},
{"LOWPOWER_DISABLE" 	, 0	, 1      , 0 	},
{"WAKEUP_ENABLE" 	, 0	, 7      , 7 	},
{"WATCHDOG_ENABLE" 	, 0	, 1      , -1 	},
{"GPIO_ENABLE" 		, 0	, 1      , -1 	},
{"LOWPOWER_ENABLE" 	, 0	, 1      , -1 	},
{"IGNITION" 		, 0	, 1      , -1 	},
{"GPIO" 		, 0	, 1      , -1 	},
{"INPUT_TEMP_MIN" 	, 0	, 23000  , 23000},
{"INPUT_TEMP_MAX" 	, 0	, 30000  , 30000},
{"INPUT_TEMP" 		, -1	, -1     , -1   },
#endif
/******************/
};

#define POWER_SATAE_MAXSTRING	20
int8_t strPowerState[][POWER_SATAE_MAXSTRING]= 
			{	"CLOSEUP",
				"POWERON_DELAY",
				"WAIT_STARTUP",
				"STARTUP",
				"SHUTDOWN_DEALY",
				"SHUTTING_DOWN",
				"LOWPOWER_DELAY" 
				};
				

int32_t aryParam[]={
(int32_t)cmdPOWERON_DELAY,
(int32_t)cmdSTARTUP_TIMEOUT,
(int32_t)cmdWATCHDOG_DEFAULT,
(int32_t)cmdWDT_RETRY_MAX,
(int32_t)cmdINPUT_VOLTAGE_MIN,
(int32_t)cmdINPUT_VOLTAGE_MAX,
(int32_t)cmdLOWPOWER_DELAY,
(int32_t)cmdFAIL_RETRY,
(int32_t)cmdSHUTDOWN_DELAY,
(int32_t)cmdSHUTDOWN_TIMEOUT,
(int32_t)cmdINPUT_TEMP_MIN,
(int32_t)cmdINPUT_TEMP_MAX
};

