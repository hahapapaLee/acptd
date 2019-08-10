/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "7seg.h"
#include "fifo.h"
//#include "eeprom_24Cxx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	uint8_t buff[10];
	uint8_t count;
	uint32_t flag;
}UartRxTypeDef;

/* Communication Check */
typedef struct
{
	uint32_t count;					// check period
	uint32_t event;					// event
}CommErrorTypeDef;

typedef struct
{
	volatile uint8_t type;
	volatile uint8_t dir;			// North, East, South, West
	volatile uint8_t PedTim;		// Pedestrian Time Type
	volatile uint8_t side;			// Left or Right
	volatile uint8_t PedSig;		// Pedestrian Signal Type
}DeviceInfoTypeDef;

typedef struct
{
	volatile uint8_t stat;
	volatile uint8_t time;
	volatile uint8_t backup;
	volatile uint8_t display; 
}PedSignalTypeDef;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifdef __GNUC__
  #define PUTCHAR_PROTOTYPE 					int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE 					int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define YES										1
#define NO										0
  
#define LED_RUN_OFF								HAL_GPIO_WritePin (LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_SET)
#define LED_RUN_ON								HAL_GPIO_WritePin (LED_RUN_GPIO_Port, LED_RUN_Pin, GPIO_PIN_RESET)
#define LED_RUN_TOGGLE							HAL_GPIO_TogglePin(LED_RUN_GPIO_Port, LED_RUN_Pin)

#define LED_TX_OFF								HAL_GPIO_WritePin (LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_SET)
#define LED_TX_ON								HAL_GPIO_WritePin (LED_ERR_GPIO_Port, LED_ERR_Pin, GPIO_PIN_RESET)
#define LED_TX_TOGGLE							HAL_GPIO_TogglePin(LED_ERR_GPIO_Port, LED_ERR_Pin)

#define LED_RX_OFF								HAL_GPIO_WritePin (LED_COM_GPIO_Port, LED_COM_Pin, GPIO_PIN_SET)
#define LED_RX_ON								HAL_GPIO_WritePin (LED_COM_GPIO_Port, LED_COM_Pin, GPIO_PIN_RESET)
#define LED_RX_TOGGLE							HAL_GPIO_TogglePin(LED_COM_GPIO_Port, LED_COM_Pin)  
  
#define RS485_SEND_ENABLE						HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_SET)
#define RS485_RECEIVE_ENABLE					HAL_GPIO_WritePin(RS485_DE_GPIO_Port, RS485_DE_Pin, GPIO_PIN_RESET)  

#define TIMER2_START							HAL_TIM_Base_Start_IT(&htim2)
#define TIMER2_STOP								HAL_TIM_Base_Stop_IT(&htim2)
#define TIMER3_START							HAL_TIM_Base_Start_IT(&htim3)
#define TIMER3_STOP								HAL_TIM_Base_Stop_IT(&htim3)

#define AC_PED_OUTPUT_ON						HAL_GPIO_WritePin(PED_OUT_GPIO_Port, PED_OUT_Pin, GPIO_PIN_SET) //modified 190729
#define AC_PED_OUTPUT_OFF						HAL_GPIO_WritePin(PED_OUT_GPIO_Port, PED_OUT_Pin, GPIO_PIN_RESET) //modified 190729

#define GET_AC_PED_INPUT						HAL_GPIO_ReadPin(PED_IN_GPIO_Port, PED_IN_Pin) //modified 190729
#define GET_STAT_OPT_SW1						HAL_GPIO_ReadPin(OPT_SW1_GPIO_Port, OPT_SW1_Pin)
#define GET_STAT_OPT_SW2						HAL_GPIO_ReadPin(OPT_SW2_GPIO_Port, OPT_SW2_Pin)
#define GET_STAT_OPT_SW3						HAL_GPIO_ReadPin(OPT_SW3_GPIO_Port, OPT_SW3_Pin)
#define GET_STAT_OPT_SW4						HAL_GPIO_ReadPin(OPT_SW4_GPIO_Port, OPT_SW4_Pin)
//#define GET_STAT_OPT_SW5						HAL_GPIO_ReadPin(OPT_SW5_GPIO_Port, OPT_SW5_Pin) //modified 190729

#define TIM_SER_H								HAL_GPIO_WritePin (TIM_SER_GPIO_Port, TIM_SER_Pin, GPIO_PIN_SET)
#define TIM_SER_L								HAL_GPIO_WritePin (TIM_SER_GPIO_Port, TIM_SER_Pin, GPIO_PIN_RESET)
  
#define TIM_SCK_H								HAL_GPIO_WritePin (TIM_SCK_GPIO_Port, TIM_SCK_Pin, GPIO_PIN_SET)
#define TIM_SCK_L								HAL_GPIO_WritePin (TIM_SCK_GPIO_Port, TIM_SCK_Pin, GPIO_PIN_RESET)
  
#define TIM_RCK_H								HAL_GPIO_WritePin (TIM_RCK_GPIO_Port, TIM_RCK_Pin, GPIO_PIN_SET)
#define TIM_RCK_L								HAL_GPIO_WritePin (TIM_RCK_GPIO_Port, TIM_RCK_Pin, GPIO_PIN_RESET)

#define TIM_NOE_H								HAL_GPIO_WritePin (TIM_NOE_GPIO_Port, TIM_NOE_Pin, GPIO_PIN_SET)
#define TIM_NOE_L								HAL_GPIO_WritePin (TIM_NOE_GPIO_Port, TIM_NOE_Pin, GPIO_PIN_RESET)

#define TimeOutEnable()							TIM_NOE_L
#define TimeOutDisable()						TIM_NOE_H

#define FLASH_PAGE_START_ADDR(n)				(uint32_t)(FLASH_BASE + (FLASH_PAGE_SIZE*n))
#define FLASH_PAGE_END_ADDR(n)					(FLASH_PAGE_START_ADDR(n) + FLASH_PAGE_SIZE)
#define FLASH_PAGE_DATA(p,d)					(FLASH_PAGE_START_ADDR(p)+4*(d))
#define FLASH_PAGE_FOR_OPTION					30
#define FLASH_ADDRESS_FOR_OPTION				FLASH_PAGE_DATA(FLASH_PAGE_FOR_OPTION, 0)
#define ERASE_FLASH_MEMORY						FlashPageErase(FLASH_PAGE_FOR_OPTION)

#define COMM_CHK_T								10

#define PED_TIME_CLEAR							TimeOutDisable()//DisplayTimeLed(0xFFFF, OFF)

#define STX										0x7E
#define CROSS_ID								0x00 // Crossroad ID
#define OP_CODE_REQ								0x9A // op-code for request
#define OP_CODE_RES								0x9B // op-code for response
#define OP_CODE_SET								0x9C // op-code for id setting

#define NORTH									0
#define EAST									4
#define SOUTH									8
#define WEST									12

#define __LED_CRT_MEASURE__
#ifdef __LED_CRT_MEASURE__
	#define RES_BYTE_SIZE						9
#else
	#define RES_BYTE_SIZE						7
#endif
#define REQ_BYTE_SIZE							9
#define SET_BYTE_SIZE							7
#define REQ_LENGTH								(REQ_BYTE_SIZE - 2)
#define SET_LENGTH								(SET_BYTE_SIZE - 2)

#define REMAIN_TIME_FOR_TEST					199
#define WAIT_TIME_FOR_TEST						10

#define PR_SIG									0x01
#define PG_SIG									0x02
#define PG_TIM									0x01
#define PR_TIM									0x02

#define MSK_STORE_DEV_ID						0x80
#define MSK_ERASE_DEV_ID						0x40
#define MSK_SYSTEM_RESET						0x20
#define MSK_AUTO_CNTDOWN						0x08 //modified 190725
#define MSK_PRTIME_LIMIT						0x10 //modified 190725
#define MSK_PED_TIME_OFF						0x80 //modified 190730

#define ADC_CHANNELS							3
#define ADC_SUM_SIZE							50

#define MAIN_VOLTAGE							g_fVoltage[0] //modified 190730
#define SCAP_VOLTAGE							g_fVoltage[2] //modified 190730
#define MAIN_VTG_THRESHOLD						4200 //modified 190730
#define SCAP_VTG_THRESHOLD						1000 //modified 190731
#define DISPLAY_LIMIT_SEC						5 //modified 190731

#define GET_PED_DIRECTION(n)					((n >> 2) & 0x03)
#define GET_PEDTIME_TYPE(n)						((n >> 1) & 0x01) == 1? PG_TIM : PR_TIM;
#define GET_PEDSIGN_TYPE(n)						((n >> 1) & 0x01) == 1? PR_SIG : PG_SIG;
#define GET_PED_POSITION(n)						(n & 0x01)

enum{RED, GRE};
enum{LEFT, RIGHT};
enum{OFF, ON};
enum{STOP, START};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t g_ucDeviceID;
volatile uint32_t g_uiTick100ms 					= 0;
uint32_t g_uiRunTimeVal 							= 1;
volatile uint8_t g_ucDebugMsgEnable					= RESET;
volatile uint8_t g_ucDebugDumpEnable				= RESET;
uint32_t g_uiTim2Tick;

// ADC
uint16_t g_ucAdcBuf[ADC_CHANNELS];
uint32_t g_uiAdcSum[ADC_CHANNELS];
uint32_t g_uiAdcSumCnt;
uint32_t g_uiCurrentSum;
uint32_t g_uiAdcAvg[ADC_CHANNELS];
float g_fVoltage[ADC_CHANNELS];
float g_fMainVtg, g_fPedLedCrt;

//float g_arrVioutQ[ADC_CHANNELS];
volatile uint32_t g_uiAdcConvCpltFlag				= RESET;
volatile uint32_t g_uiCrtSumCnt						= 0;

#if 1
const uint8_t g_arrFndSeg[] 						= {SEG_0, SEG_1, SEG_2, SEG_3, SEG_4, SEG_5, SEG_6, SEG_7, SEG_8, SEG_9};
#else
const uint8_t g_arrFndSeg[] 						= {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x27, 0x7F, 0x6F};
#endif

uint8_t g_arrIdIndex[16]							= {0, 1, 0, 1, 4, 5, 4, 5, 8, 9, 8, 9, 12, 13, 12, 13};
uint8_t g_arrRxProtocol[5]							= {STX,  STX, 7, CROSS_ID, OP_CODE_REQ};
uint8_t g_arrTxProtocol[RES_BYTE_SIZE];
uint8_t g_arrSettingProtocol[5]						= {STX,  STX, 5, CROSS_ID, OP_CODE_SET};

char* g_strDir[4]									= {"NORTH", "EAST ", "SOUTH", "WEST "};
char* g_strTimType[3]								= {" ","PG_TIM","PR_TIM"};
char* g_strSigType[3]								= {" ","PR_SIG","PG_SIG"};
char* g_strSide[2]									= {"LEFT ", "RIGHT"};
char* g_strDisplay[2]								= {"OFF", " ON"};

volatile uint8_t g_ucValidDataFlag 					= RESET;
volatile uint8_t g_ucSettingDevFlag					= RESET;

// added 190311
volatile uint32_t g_uiCountDownTick					= 0;
volatile uint8_t g_ucTimeCntdnEvent					= RESET;
volatile uint8_t g_ucAutoRunTimeCntdn				= STOP;
volatile uint8_t g_ucAutoCntdnTime					= 0;
volatile uint8_t g_ucAutoCntdnMode					= RESET;
volatile uint8_t g_ucPRTimeLimitMode				= RESET;
volatile uint8_t g_ucPedTimeOffMode					= RESET; //modified 190730

volatile uint8_t g_ucExtPedSta, g_ucExtPedStaBak 	= 0xFF; //modified 190729

uint8_t g_ucUart1rData;
FIFOTypeDef g_Uart1Fifo, g_Uart2Fifo;
UartRxTypeDef g_Uart1Rx, g_Uart2Rx;
CommErrorTypeDef g_CommErr 							= {COMM_CHK_T, RESET};
DeviceInfoTypeDef g_DeviceInfo;
PedSignalTypeDef g_PedSignal						= {0,0xFF,0xFF,RESET};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
//static uint8_t ReadDevId(void);
__STATIC_INLINE void delay_100usec(uint32_t n);
__STATIC_INLINE void DisplayTimeLed (uint32_t val, uint32_t enable);
__STATIC_INLINE void SendDataVia485(uint8_t* ch, uint16_t size);
__STATIC_INLINE uint8_t xor_sum(uint8_t* pData, uint32_t size);
static void FlashPageErase(uint32_t page);
static uint32_t FlashRead(uint32_t address);
static uint32_t FlashWrite(uint32_t address, uint32_t data);
static void Rx485DataProc(void);
static void TxDataProc(void);
static void AdcConvProc(void);
static void IdSettingFromRS485(void);
static void IdSettingFromRS232(void);
static void SelfTestMode(void);

static void TitleMessages(void)
{
	#define MAJOR_VER	1
	#define MINOR_VER	2
	#define DATE		"2019.07.31"

	// Print message on the terminal
	printf("\n\n\n\r*************************************************");
	printf(    "\n\r SUHDOL Electronic & Communication Co,. Ltd");
	printf(    "\n\r Ver   : %d.%d",MAJOR_VER,MINOR_VER);
	printf(    "\n\r Date  : %s",DATE);
	printf(    "\n\r*************************************************\n\r");
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	uint32_t uiFlashBuff = 0;
	memset(&g_Uart1Rx, 0, sizeof(g_Uart1Rx));
	memset(&g_uiAdcSum, 0, sizeof(g_uiAdcSum)); //modified 190724

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_IWDG_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

	PED_TIME_CLEAR;
	TitleMessages();

	if((uiFlashBuff = FlashRead(FLASH_ADDRESS_FOR_OPTION)) != 0xFFFFFFFF)
	{
		g_ucDeviceID = (uint8_t)(0x0F & uiFlashBuff); //modified 190724
		g_DeviceInfo.dir    = GET_PED_DIRECTION(g_ucDeviceID);
		g_DeviceInfo.PedTim = GET_PEDTIME_TYPE(g_ucDeviceID);
		g_DeviceInfo.PedSig = GET_PEDSIGN_TYPE(g_ucDeviceID);
		g_DeviceInfo.side   = GET_PED_POSITION(g_ucDeviceID);
	//	g_ucAutoCntdnMode   = (uint8_t)(MSK_AUTO_CNTDOWN & uiFlashBuff) >> 4; //modified 190724 -> removed 190725

		printf("\n\r Device ID[0x%02x]",g_ucDeviceID);
		printf("\n\r > Direction: %d\t\t-> %s",g_DeviceInfo.dir,g_strDir[g_DeviceInfo.dir]);
		printf("\n\r > Side     : %d\t\t-> %s",g_DeviceInfo.side,g_strSide[g_DeviceInfo.side]);
		printf("\n\r > Ped  Type: %d\t\t-> %s",g_DeviceInfo.PedSig,g_strSigType[g_DeviceInfo.PedSig]);
		printf("\n\r > Time Type: %d\t\t-> %s",g_DeviceInfo.PedTim,g_strTimType[g_DeviceInfo.PedTim]);
	}
	else
	{
		if(!GET_STAT_OPT_SW2)	IdSettingFromRS232();
		else					IdSettingFromRS485();
	}

	if(!GET_STAT_OPT_SW3) g_ucDebugMsgEnable = SET;
	if(!GET_STAT_OPT_SW4) SelfTestMode();
//	if(!GET_STAT_OPT_SW5) g_ucAutoCntdnMode = SET; //modified 190724

//	printf("\n\r > Time Countdown Mode\t-> %s", g_ucAutoCntdnMode == SET?"AUTO":"COMM"); // removed 190725
	printf("\n\r > Device Setting Mode\t-> %s", GET_STAT_OPT_SW2 == 0?"RS-232":"RS-485");

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_UART_Receive_DMA(&huart1, &g_ucUart1rData, 1);
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)g_ucAdcBuf, ADC_CHANNELS);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

		HAL_IWDG_Refresh(&hiwdg);

		if(FifoIsEmpty(&g_Uart1Fifo) != YES)
		{
			Rx485DataProc();
		}

		if(g_ucValidDataFlag)
		{
			g_ucValidDataFlag = RESET;

			g_CommErr.count = COMM_CHK_T;
			g_CommErr.event = RESET;

/*			if(g_ucDebugMsgEnable) 
			{
				printf("\n\n\r Rx Data:");
				for(uint8_t i = 0; i < REQ_BYTE_SIZE; i++) printf(" %02x", g_Uart1Rx.buff[i]);
			}*/

			g_PedSignal.stat = g_Uart1Rx.buff[6];
			g_PedSignal.time = g_Uart1Rx.buff[7];

			g_ucPedTimeOffMode = ((g_PedSignal.stat & MSK_PED_TIME_OFF) == 0x80)? SET : RESET; //modified 190730
			g_ucPRTimeLimitMode = ((g_PedSignal.stat & MSK_PRTIME_LIMIT) == 0x10)? SET : RESET; //modified 190725
			g_ucAutoCntdnMode = ((g_PedSignal.stat & MSK_AUTO_CNTDOWN) == 0x08)? SET : RESET; //modified 190725

			if((g_PedSignal.stat & 0x07) == g_DeviceInfo.PedTim) //modified 190725
			{
				if(g_PedSignal.time > 0 && g_PedSignal.time < 200) 
				{
					g_PedSignal.display = ON;
				}
				else 
				{
					g_PedSignal.time = 0;
					g_PedSignal.display = OFF;
				}
			}
			else
			{
				g_PedSignal.time = 0;
				g_PedSignal.display = OFF;
			}

			/*-------------------------------- modified 190725 ------------------------------*/
			TxDataProc();

			if(g_ucDebugMsgEnable) 
			{
				printf("\n\n\r Rx Data:");
				for(uint8_t i = 4; i < REQ_BYTE_SIZE-1; i++) printf(" %02x", g_Uart1Rx.buff[i]);
				printf("\t=> [%s][%s][%s][%3d][%s]",\
									g_ucAutoCntdnMode == SET?"AUTO":"COMM",\
									(g_PedSignal.stat & 0x07) == 0x01?"PG":"PR",\
									g_strDisplay[g_PedSignal.display],\
									g_PedSignal.time,\
									g_ucPRTimeLimitMode == SET?"LMT":"NOLMT");
				printf("\n\r Tx Data:");
				for(uint8_t i = 4; i < RES_BYTE_SIZE-1; i++) printf(" %02x", g_arrTxProtocol[i]);
			}
			/*-----------------------------------------------------------------------------------*/
		}
		
		if(g_ucSettingDevFlag)
		{
			g_ucSettingDevFlag = RESET;

			if((g_Uart1Rx.buff[5] & MSK_ERASE_DEV_ID) == 0x40)
			{
				ERASE_FLASH_MEMORY;
				if(g_ucDebugMsgEnable) printf("\n\r Initialize the Device ID!!");
				NVIC_SystemReset();
			}
			else if((g_Uart1Rx.buff[5] & MSK_SYSTEM_RESET) == 0x20)
			{
				if(g_ucDebugMsgEnable) printf("\n\r System Reset!!");
				NVIC_SystemReset();
			}
		}

		if(g_ucExtPedSta != 0x00)
		{
			if(g_ucExtPedSta != g_ucExtPedStaBak) 
			{
				PED_TIME_CLEAR;
				g_ucAutoRunTimeCntdn = STOP; // added 190311
				g_ucAutoCntdnTime = 0; // added 190311
			//	if(g_ucDebugMsgEnable) printf("\n\r %s ON",g_strSigType[g_DeviceInfo.PedSig]); //modified 190729
				g_uiCurrentSum = 0;
				g_uiCrtSumCnt = 0;
			}
		}
		else // No pedestrian signal
		{
			/* modified 190729
			if(g_ucExtPedSta != g_ucExtPedStaBak) 
			{
				if(g_ucDebugMsgEnable) printf("\n\r %s OFF",g_strSigType[g_DeviceInfo.PedSig]);
			}*/

			if(g_ucPedTimeOffMode || SCAP_VOLTAGE < SCAP_VTG_THRESHOLD) g_PedSignal.display = OFF; //modified 190730

			if(g_ucAutoCntdnMode)
			{
				if(g_ucAutoRunTimeCntdn == STOP && g_PedSignal.display == ON)
				{
					g_uiCountDownTick = 0;
					g_ucAutoRunTimeCntdn = START;
					g_ucAutoCntdnTime = g_PedSignal.time; // set the initial-countdown-time-value
				}
				else if(g_ucAutoRunTimeCntdn == START && g_PedSignal.display == OFF)
				{
					g_uiCountDownTick = 0;
					g_ucAutoRunTimeCntdn = STOP;
					g_ucAutoCntdnTime = g_PedSignal.time;
					DisplayTimeLed(g_ucAutoCntdnTime, g_PedSignal.display);
				}
				
				if(g_ucTimeCntdnEvent)
				{
					g_ucTimeCntdnEvent = RESET;
					if(g_ucAutoCntdnTime > 1) 
					{
						g_ucAutoCntdnTime--;
						if(g_DeviceInfo.PedTim == 0x02 && g_ucPRTimeLimitMode && g_ucAutoCntdnTime < DISPLAY_LIMIT_SEC) g_PedSignal.display = OFF; //modified 190725
						DisplayTimeLed(g_ucAutoCntdnTime, g_PedSignal.display);
					}
					else
					{
						g_PedSignal.display = OFF;
						g_PedSignal.time = 0;
						g_ucAutoRunTimeCntdn = STOP;
						DisplayTimeLed(g_ucAutoCntdnTime, g_PedSignal.display);
					}
					if(g_ucDebugMsgEnable && g_PedSignal.display) 
					{
						printf("\t=> [%s][AutoCnt][%3d]",g_PedSignal.time == g_ucAutoCntdnTime? "SAME":"DIFF",g_ucAutoCntdnTime);
					//	printf("\n\r MAIN_V[mV]:%1.0f, SCAP_V[mV]:%1.0f, PED_V[mV]:%1.0f -> %1.2fmA ", g_fVoltage[0],g_fVoltage[2],g_fVoltage[1],g_fPedLedCrt*10); //modified 190724
					}
				}
			}
			else
			{
				if(g_DeviceInfo.PedTim == 0x02 && g_ucPRTimeLimitMode && g_PedSignal.time < DISPLAY_LIMIT_SEC) g_PedSignal.display = OFF; //modified 190725
				if(g_PedSignal.time != g_PedSignal.backup) 
				{
					g_PedSignal.backup = g_PedSignal.time;
					DisplayTimeLed(g_PedSignal.time, g_PedSignal.display);
				}
			}
		}
		g_ucExtPedStaBak = g_ucExtPedSta; //modified 190726

		if(g_CommErr.event)
		{
			g_CommErr.event = RESET;

		//	PED_TIME_CLEAR; //modified 190311
			LED_TX_OFF;
			LED_RX_OFF;
		}

		if(g_uiAdcConvCpltFlag) //modified 190724
		{
			g_uiAdcConvCpltFlag = RESET;
			AdcConvProc(); //modified 190730
		}

		if(g_uiTick100ms >= 5) // period:500msec
		{
			g_uiTick100ms = 0;
			HAL_UART_Receive_DMA(&huart1, &g_ucUart1rData, 1);
			g_ucDebugMsgEnable = !GET_STAT_OPT_SW3;
			g_ucDebugDumpEnable = !GET_STAT_OPT_SW4;
			
		#if 1
			if(g_ucDebugMsgEnable) printf("\n\r MAIN_V:SCAP_V:PED_C\t=> [%1.0fmV][%1.0fmV][%1.2fmA]", g_fVoltage[0],g_fVoltage[2],g_fPedLedCrt*10);
		#else
			if(g_ucDebugMsgEnable) printf("\n\r MAIN_V[mV]:%1.0f, SCAP_V[mV]:%1.0f, PED_V[mV]:%1.0f -> %1.2fmA ", g_fVoltage[0],g_fVoltage[2],g_fVoltage[1],g_fPedLedCrt*10);
		#endif
		}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
////////////////////////////////////////////////////////////// Prototype Function ////////////////////////////////////////////////////////////////////
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE        // USART3
{
	uint8_t temp[1] = {ch};
	/* Write a character to the USART */
	HAL_UART_Transmit(&huart3, temp, 1, 2);

	return ch;
}
////////////////////////////////////////////////////////////// Callback Functions ////////////////////////////////////////////////////////////////////
void HAL_SYSTICK_Callback(void)
{
	static uint32_t uiTicks, uiRunLedTick;
	/*----------modified 190729--------------------*/
	g_ucExtPedSta <<= 1; // solution for dimming
	if(GET_AC_PED_INPUT == ON) g_ucExtPedSta |= 0x01;
	if(g_ucExtPedSta != 0x00 && MAIN_VOLTAGE > MAIN_VTG_THRESHOLD)
		AC_PED_OUTPUT_ON;
	else 
		AC_PED_OUTPUT_OFF;
	/*---------------------------------------------*/

	uiTicks++;
	// added 190311
	if(g_ucAutoRunTimeCntdn == START)
	{
		g_uiCountDownTick++; 
		if(g_uiCountDownTick == 1000)
		{
			g_uiCountDownTick = 0;
			g_ucTimeCntdnEvent = SET;
		}
	}

	if(uiTicks == 100)					// T=100ms
	{
		uiTicks = 0;
		g_uiTick100ms++;
		uiRunLedTick++;

		if(g_CommErr.count == 0)
		{
			g_CommErr.event = SET;
			g_uiRunTimeVal = 10;
		}
		else
		{
			g_CommErr.count--;
			g_uiRunTimeVal = 5;
		}
	}

	if(uiRunLedTick >= g_uiRunTimeVal)	// Normal:500ms, Error:1000ms
	{
		uiRunLedTick = 0;
		LED_RUN_TOGGLE;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim2)
	{
		g_uiTim2Tick++;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		Fifo_EnQueue(&g_Uart1Fifo, g_ucUart1rData);
		LED_RX_TOGGLE;

		if(HAL_UART_Receive_DMA(&huart1, &g_ucUart1rData, 1) != HAL_OK)
		{
			if(g_ucDebugMsgEnable) printf("\n\r UART Rx Interrupt Error!!!\n\r");
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1)
	{
		LED_TX_TOGGLE;
#define __USE_UART_TX_DMA__
#ifdef __USE_UART_TX_DMA__
		RS485_RECEIVE_ENABLE;
#endif
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc == &hadc1)
	{
		g_uiAdcConvCpltFlag = SET;
	}
}
////////////////////////////////////////////////////////////// User Functions //////////////////////////////////////////////////////////////////
/*
uint8_t ReadDevId(void)
{
	uint32_t id, val;

	val = (GPIOB->IDR & 0xE0)>>5;

	if(HAL_GPIO_ReadPin(DEV_ID3_GPIO_Port, DEV_ID3_Pin) == GPIO_PIN_SET) val |= 0x8;

	id = 0x0F & (~val);			// real-code

	g_DeviceInfo.dir   = id & 0xC0;
	g_DeviceInfo.PedTim = id & 0x02;
	g_DeviceInfo.side  = id & 0x01;

	return g_arrIdIndex[id];
}
*/
__STATIC_INLINE void DisplayTimeLed (uint32_t val, uint32_t enable)
{
	uint16_t usIdx, usData, usHigh, usLow;

	if(enable == ON)
	{
		if(val < 100) usHigh = val / 10;
		else usHigh = (val - 100) / 10;

		usLow = val % 10;

		if(usHigh != 0 || val >= 100) usData = (g_arrFndSeg[usHigh]<<8) | g_arrFndSeg[usLow];
		else usData = 0xFF00 | g_arrFndSeg[usLow];

		if(val >= 100)  usData &= 0x7FFF;
		else usData |= 0x8000;
	}
	else
	{
		usData = 0xFFFF;
	}

	TimeOutEnable();

	for(usIdx = 16; usIdx != 0; usIdx--)
	{
		TIM_SCK_L;
	//	HAL_Delay(1);
		delay_100usec(1);

		switch(usData & 0x8000)
		{
			case 0x8000 :
				TIM_SER_L;
				break;
			default:
				TIM_SER_H;
				break;
		}
		TIM_SCK_H;
	//	HAL_Delay(1);
		delay_100usec(1);

		usData = usData << 1;
	}

	TIM_RCK_H;
//	HAL_Delay(1);
	delay_100usec(1);
	TIM_RCK_L;
}

__STATIC_INLINE void SendDataVia485(uint8_t* ch, uint16_t size)
{
	/* Write a character to the USART */
	RS485_SEND_ENABLE;

#ifndef __USE_UART_TX_DMA__ 
	HAL_UART_Transmit(&huart1, ch, size, 2);
	HAL_Delay(1);
	RS485_RECEIVE_ENABLE;
#else
	HAL_UART_Transmit_DMA(&huart1, ch, size);
#endif
}

__STATIC_INLINE void delay_100usec(uint32_t n)
{
	HAL_TIM_Base_Start_IT(&htim2);
	while(g_uiTim2Tick < n)
	{
		HAL_IWDG_Refresh(&hiwdg);
	}
	HAL_TIM_Base_Stop_IT(&htim2);
	g_uiTim2Tick = 0;
}

__STATIC_INLINE uint8_t xor_sum(uint8_t* pData, uint32_t size)
{
	uint8_t ret = *pData;

	for(int i = 1; i < size-1 ; i++)
	{
		ret ^= *(pData + i);
	}

	return ret;
}

uint32_t FlashRead(uint32_t address)
{
	return *(__IO uint32_t*)address;
}

uint32_t FlashWrite(uint32_t Address, uint32_t data)
{
	if(FlashRead(Address) != 0xFFFFFFFF) return ERROR;

	HAL_FLASH_Unlock();

	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, ((uint32_t)data)) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_FLASH_Lock();

	return SUCCESS;
}

static void FlashPageErase(uint32_t page)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t g_uiSectorError = 0;

	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = FLASH_PAGE_START_ADDR(page);
	EraseInitStruct.NbPages = ((FLASH_PAGE_START_ADDR(page) + FLASH_PAGE_SIZE) - FLASH_PAGE_START_ADDR(page))/FLASH_PAGE_SIZE;

	HAL_FLASH_Unlock();

	if(HAL_FLASHEx_Erase(&EraseInitStruct, &g_uiSectorError) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_FLASH_Lock();
}

static void TxDataProc(void)
{
	g_arrTxProtocol[0] = STX;
	g_arrTxProtocol[1] = STX;
#ifndef __LED_CRT_MEASURE__
	g_arrTxProtocol[2] = 5;
#else
	g_arrTxProtocol[2] = 7;
#endif
	g_arrTxProtocol[3] = CROSS_ID;
	g_arrTxProtocol[4] = OP_CODE_RES;
	g_arrTxProtocol[5] = g_ucDeviceID;
#ifndef __LED_CRT_MEASURE__
	g_arrTxProtocol[6] = xor_sum(g_arrTxProtocol, RES_BYTE_SIZE);
#else
	g_arrTxProtocol[6] = (uint8_t)g_fPedLedCrt;
	g_arrTxProtocol[7] = (uint8_t)g_fMainVtg;
	g_arrTxProtocol[8] = xor_sum(g_arrTxProtocol, RES_BYTE_SIZE);
#endif
	SendDataVia485(g_arrTxProtocol, RES_BYTE_SIZE);
}

/*---------------- modified 190730 --------------------*/
#define __DUMP_MSG_COUNT__
#ifdef __DUMP_MSG_COUNT__
static void DumpMsgCounter(uint8_t* value)
{
	uint8_t temp;
	temp = *value;
	temp++;
	if(temp > 8) 
	{
		printf("\n\r");
		temp = 0;
	}
	*value = temp;
}
#endif

static void Rx485DataProc(void)
{
	uint8_t ucTmpBuff = Fifo_DeQueue(&g_Uart1Fifo);
#ifdef __DUMP_MSG_COUNT__
	static uint8_t ucMsgCnt;
#endif

	switch(g_Uart1Rx.count)
	{
		case 0:
			if(ucTmpBuff == STX) 
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else 
			{
				g_Uart1Rx.count = 0;
			}

			if(g_ucDebugDumpEnable)
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 1:
			if(ucTmpBuff == STX) 
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else 
			{
				g_Uart1Rx.count = 0;
			}

			if(g_ucDebugDumpEnable) 
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 2:
			if(ucTmpBuff == REQ_LENGTH) 
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else if(ucTmpBuff == SET_LENGTH)
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else 
			{
				g_Uart1Rx.count = 0;
			}

			if(g_ucDebugDumpEnable) 
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 3:
			if(ucTmpBuff == CROSS_ID) 
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else 
			{
				g_Uart1Rx.count = 0;
			}

			if(g_ucDebugDumpEnable) 
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 4:
			if(ucTmpBuff == OP_CODE_REQ)
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else if(ucTmpBuff == OP_CODE_SET)
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else 
			{
				g_Uart1Rx.count = 0;
			}
			
			if(g_ucDebugDumpEnable) 
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 5:
			if(ucTmpBuff == g_ucDeviceID) 
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else if(g_Uart1Rx.buff[4] == OP_CODE_SET)
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else 
			{
				g_Uart1Rx.count = 0;
			}
			
			if(g_ucDebugDumpEnable)
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 6:
			if(g_Uart1Rx.buff[2] == REQ_LENGTH)
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else if(g_Uart1Rx.buff[4] == OP_CODE_SET && ucTmpBuff == xor_sum(g_Uart1Rx.buff, SET_BYTE_SIZE))
			{
				g_Uart1Rx.count = 0;
				g_ucSettingDevFlag = SET;
			}
			else
			{
				g_Uart1Rx.count = 0;
			}
			
			if(g_ucDebugDumpEnable) 
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 7:
			if(g_Uart1Rx.buff[2] == REQ_LENGTH)
			{
				g_Uart1Rx.buff[g_Uart1Rx.count] = ucTmpBuff;
				g_Uart1Rx.count++;
			}
			else 
			{
				g_Uart1Rx.count = 0; 
			}

			if(g_ucDebugDumpEnable) 
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x ",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		case 8:
			if(ucTmpBuff == xor_sum(g_Uart1Rx.buff, REQ_BYTE_SIZE)) 
			{
				g_ucValidDataFlag = SET;
			}
			g_Uart1Rx.count = 0;

			if(g_ucDebugDumpEnable) 
			{
			#ifdef __DUMP_MSG_COUNT__
				DumpMsgCounter(&ucMsgCnt);
			#endif
				printf("(%d)%02x\n\r",g_Uart1Rx.count,ucTmpBuff);
			}
			break;
		default: 
			break;
	}
}

static void AdcConvProc(void)
{
	uint8_t i;
	for(i = 0; i < ADC_CHANNELS; i++) g_uiAdcSum[i] += g_ucAdcBuf[i];
	
	g_uiAdcSumCnt++;
	if(g_uiAdcSumCnt == ADC_SUM_SIZE)
	{
		g_uiAdcSumCnt = 0;
		for(i = 0; i < ADC_CHANNELS; i++) g_uiAdcAvg[i] = g_uiAdcSum[i]/ADC_SUM_SIZE;
		memset(&g_uiAdcSum, 0, sizeof(g_uiAdcSum));

		g_fVoltage[0] = (g_uiAdcAvg[0]*6600/4096); //unit:mV, 3.3V*2
		g_fVoltage[1] = (g_uiAdcAvg[1]*3300/4096);
		g_fVoltage[2] = (g_uiAdcAvg[2]*6600/4096); //unit:mV, 3.3V*2

		g_fMainVtg = g_fVoltage[0]/100; // uint:100mV
		g_fPedLedCrt = g_fVoltage[1]*2/100; //unit:10mA
	}
}
/*--------------------------------------------------------*/
static void IdSettingFromRS485(void)
{
	uint8_t ucExitLoop = RESET;
	uint8_t ucDevID = 0xFF;
	uint8_t ucDisplayDevId = RESET;
	uint8_t ucToggleFlag = RESET;

	HAL_UART_Receive_DMA(&huart1, &g_ucUart1rData, 1);
	printf("\n\n\r Setting Device ID From RS485 Protocol...");
	MAIN_VOLTAGE = MAIN_VTG_THRESHOLD - 1; //modified 190730
	do
	{
		/*--------modified 190730-------*/
		if(g_ucExtPedSta != 0x00)
		{
			MAIN_VOLTAGE++;
			if(MAIN_VOLTAGE > MAIN_VTG_THRESHOLD) MAIN_VOLTAGE = MAIN_VTG_THRESHOLD + 1;
		}
		else MAIN_VOLTAGE = MAIN_VTG_THRESHOLD - 1;
		/*------------------------------*/
		HAL_IWDG_Refresh(&hiwdg);

		if(FifoIsEmpty(&g_Uart1Fifo) != YES)
		{
			Rx485DataProc();
		}
		
		if(g_ucSettingDevFlag)
		{
			g_ucSettingDevFlag = RESET;

			if((g_Uart1Rx.buff[5] & MSK_STORE_DEV_ID) == 0x80) // Store data in the internal flash memory
			{
				if(ucDevID != 0xFF)
				{
					FlashWrite(FLASH_ADDRESS_FOR_OPTION, ucDevID); //modified 190725
					ucExitLoop = SET;
					printf("\n\r Store Device ID in the internal flash memory!!");
				}
				else
				{
					printf("\n\r Failed to store Device ID[%02x]!!",ucDevID);
				}
			}
			else if((g_Uart1Rx.buff[5] & 0xF0) == 0x00)
			{
				ucDevID = g_Uart1Rx.buff[5] & 0x0F;
				if(!GET_STAT_OPT_SW1) ucDevID += 2; // Top Controller(PG Time)

				g_DeviceInfo.dir    = GET_PED_DIRECTION(ucDevID);
				g_DeviceInfo.PedTim = GET_PEDTIME_TYPE(ucDevID);
				g_DeviceInfo.PedSig = GET_PEDSIGN_TYPE(ucDevID);
				g_DeviceInfo.side   = GET_PED_POSITION(ucDevID);
				ucDisplayDevId = SET;
				printf("\n\n\r Device ID: 0x%02x(%s/%s/%s/%s)",\
							ucDevID, g_strDir[g_DeviceInfo.dir],\
							g_strSigType[g_DeviceInfo.PedSig],\
							g_strTimType[g_DeviceInfo.PedTim],\
							g_strSide[g_DeviceInfo.side]);
			}
			else
			{
				printf("\n\r Bad data[%02x]!!", g_Uart1Rx.buff[5]);
			}
		}

		if(g_uiTick100ms >= 5)
		{
			g_uiTick100ms = 0;
			HAL_UART_Receive_DMA(&huart1, &g_ucUart1rData, 1);
			if(ucDisplayDevId)
			{
				if(!ucToggleFlag) DisplayTimeLed(ucDevID, ON);
				else PED_TIME_CLEAR;
				ucToggleFlag = !ucToggleFlag;
			}
		}
	}while(!ucExitLoop);

	printf("\n\n\r Setting complete and Reboot!!! \n\r");
	NVIC_SystemReset();
}

static void IdSettingFromRS232(void)
{
	uint8_t ucKeyBuff = 0xFF;
	uint8_t ucDevID = 0xFF;
	uint8_t ucKeyIdx = 0, ucExitLoop = 0;
	printf("\n\n\r Setting Device ID From RS232 Terminal...");
	printf("\n\r +-----------+--------------+---------+");
	printf("\n\r | Direction |  Time Color  |  Side   |");
	printf("\n\r +-----------+--------------+---------+");
	printf("\n\r |  (N)orth  |    (R)ed     | (L)eft  |");
	printf("\n\r |  (E)ast   |    (G)reen   | (R)ight |");
	printf("\n\r |  (S)outh  |              |         |");
	printf("\n\r |  (W)est   |              |         |");
	printf("\n\r +-----------+--------------+---------+");
	printf("\n\r ### [0] N/R/L [1] N/R/R [2] N/G/L [3] N/G/R");
	printf("\n\r ### [4] E/R/L [5] E/R/R [6] E/G/L [7] E/G/R");
	printf("\n\r ### [8] S/R/L [9] S/R/R [a] S/G/L [b] S/G/R");
	printf("\n\r ### [c] W/R/L [d] W/R/R [e] W/G/L [f] W/G/R");
	printf("\n\r ### Enter the Device ID: ");

	MAIN_VOLTAGE = MAIN_VTG_THRESHOLD - 1; //modified 190730
	do
	{
		/*--------modified 190730-------*/
		if(g_ucExtPedSta != 0x00)
		{
			MAIN_VOLTAGE++;
			if(MAIN_VOLTAGE > MAIN_VTG_THRESHOLD) MAIN_VOLTAGE = MAIN_VTG_THRESHOLD + 1;
		}
		else MAIN_VOLTAGE = MAIN_VTG_THRESHOLD - 1;
		/*------------------------------*/

		HAL_IWDG_Refresh(&hiwdg);
		HAL_UART_Receive(&huart3, &ucKeyBuff, 1, 100); 
		if(ucKeyBuff != 0xFF)
		{
			printf("%c", ucKeyBuff);
			if(ucKeyIdx == 0)
			{
				if(ucKeyBuff >= '0' && ucKeyBuff <= '9' ) ucDevID  = ucKeyBuff - 0x30;
				else if(ucKeyBuff >= 'A' && ucKeyBuff <= 'F' ) ucDevID  = ucKeyBuff - 0x37;
				else if(ucKeyBuff >= 'a' && ucKeyBuff <= 'f' ) ucDevID  = ucKeyBuff - 0x57;

				if(ucDevID <= 0x0F)
				{
					g_DeviceInfo.dir    = GET_PED_DIRECTION(ucDevID); 
					g_DeviceInfo.PedTim = GET_PEDTIME_TYPE(ucDevID);
					g_DeviceInfo.PedSig = GET_PEDSIGN_TYPE(ucDevID);
					g_DeviceInfo.side   = GET_PED_POSITION(ucDevID);
					printf("\n\n\r Device ID: 0x%02x(%s/%s/%s/%s)",\
								ucDevID, g_strDir[g_DeviceInfo.dir],\
								g_strSigType[g_DeviceInfo.PedSig],\
								g_strTimType[g_DeviceInfo.PedTim],\
								g_strSide[g_DeviceInfo.side]);
					printf("\n\n\r Is it correct? (Y/y or N/n)");
					ucKeyIdx = 1;
				}
			}
			else
			{
				if(ucKeyBuff == 'y' || ucKeyBuff == 'Y') 
				{
					FlashWrite(FLASH_ADDRESS_FOR_OPTION, ucDevID);
					printf("\n\r Save the device ID to internal flash memory!!");
				}
				else
				{
					printf("\n\r Not save the device ID to internal flash memory!!");
				}
				ucExitLoop = 1;
			}
			ucKeyBuff = 0xFF;
		}
	}while(!ucExitLoop);

//	g_ucDeviceID = (0xFF) & (uint8_t)FlashRead(FLASH_ADDRESS_FOR_OPTION);

	NVIC_SystemReset();
}

/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */

static void SelfTestMode(void)
{
	uint32_t uiTmpTime = 0;
	uint8_t ucTmpMode = ON;

//	GPIO_InitTypeDef GPIO_InitStruct;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(PED_IN_GPIO_Port, &GPIO_InitStruct);

	printf("\n\r============== Start Self Test Mode =======================\n\r");

	while(1)
	{
		HAL_IWDG_Refresh(&hiwdg);

		if(g_uiTick100ms >= 5)
		{
			g_uiTick100ms = 0;

			if(uiTmpTime == 0)
			{
				if(ucTmpMode == OFF) 
				{
					uiTmpTime = REMAIN_TIME_FOR_TEST;
					AC_PED_OUTPUT_OFF;
				//	HAL_GPIO_WritePin(PED_IN_GPIO_Port, PED_IN_Pin, GPIO_PIN_RESET);
				}
				else 
				{
					AC_PED_OUTPUT_ON;
					uiTmpTime = WAIT_TIME_FOR_TEST;
				//	HAL_GPIO_WritePin(PED_IN_GPIO_Port, PED_IN_Pin, GPIO_PIN_SET);
				}
				ucTmpMode = !ucTmpMode;
			}

			if(ucTmpMode)
			{
				DisplayTimeLed(uiTmpTime, ON);
				printf("\n\r Remaining Time = %3d", uiTmpTime);
			}
			else
			{
				DisplayTimeLed(0x0, OFF);
				printf("\n\r Waiting Time = %3d", uiTmpTime);
			}

			uiTmpTime--;
		}
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	printf("\n\rError: file %s on line %d\r\n", __FILE__, __LINE__);
	while(1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
