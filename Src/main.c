/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>

#include "eeprom.h"
//#include "stm32f0xx_flash.h"

#include "usbd_cdc_if.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//------------------------------------------------------------------------------
//Dígitos "BCD" a display
//Bits y su LED	  BAFGEDCp
#define	uno		0b10000010
#define dos		0b11011100
#define tres	0b11010110
#define cuatro	0b10110010
#define cinco	0b01110110
#define seis	0b01111110
#define siete	0b11000010
#define ocho	0b11111110
#define nueve	0b11110010
#define cero	0b11101110
#define nada	0
//*****************************************************************************

#define ENCODER	TIM1->CNT



#define V_Set	TIM2->CCR1
#define I_Set	TIM14->CCR1

#define BuzzerPWM	TIM2->CCR4

#define DigitsDim	TIM17->CCR1
#define LEDsDim		TIM16->CCR1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

//****************************************************************************************
//****************************************************************************************


//------------------------------------------------------------------------------
// Estructura de BCD de los 4 dígitos de display, y de valor completo para procesar
union BCD {
	unsigned long completo:32;

	struct {
		unsigned char mil:8;
		unsigned char cen:8;
		unsigned char dec:8;
		unsigned char uni:8;
	};
};

union BCD DispV;
union BCD DispC;
union BCD Process;
//****************************************************************************************
//****************************************************************************************
unsigned char flg_lessThan10, flg_IlessThan10;
unsigned char uc_NOP;
unsigned char uc_Digit;
unsigned char show;
int intShow;
//------------------------------------------------------------------------------
//Variables de tiempos (flgs y contadores de systick)
unsigned char uc_timer;	//Contador de 100ms
unsigned char uc_RD;		//Contador de refresco de Displays (10ms)
unsigned char flg_tick;	//Flag que llegó a 100ms
unsigned char flg_RD;	//Flag de refresco de display
unsigned char flg_TS;

unsigned int ui_contadorDisplay;

struct{
	int32_t Valor;

	uint32_t Vabsolute;
	uint32_t Iabsolute;

	uint16_t Vcount;
	uint16_t Icount;

	uint16_t CalcCount;
	uint16_t CalcCountAnt;

	int16_t aux;
	uint16_t multiplier;
	uint16_t vel;
	uint16_t timeout;

	uint8_t state;
} QuadEnc;

#define EncoderVoltage		1
#define EncoderCurrent		2
//****************************************************************************************
//****************************************************************************************

//------------------------------------------------------------------------------
//Variables de CDC Serial USB
unsigned char e_timeout;

uint8_t Buffer[200];
uint8_t RxBuf[50], RxMsj[50];
char flg_received, data;
unsigned char length, msjlength;

struct {
	uint8_t Message[50];

	uint8_t data;

	uint8_t flg_data:1;
	uint8_t flg_newMsj:1;
	uint8_t count:6;

} UART_RX;

//****************************************************************************************
//****************************************************************************************

//------------------------------------------------------------------------------
// Variables de conversión ADC y DAC

struct {
	char flg_newdata:1;
	char moreflags:7;

	long VSet;
	int VSet_Avg;
	float f_VSet;

	long VFb;
	int VFb_Avg;
	float f_Vfb;

	long VFb2;
	int VFb2_Avg;
	float f_Vfb2;

	long ISet;
	int ISet_Avg;
	float f_ISet;

	long IFb;
	int IFb_Avg;
	float f_Ifb;

	long Temp;
	int Temp_Avg;
	float f_Temp;

	long IntTemp;
	int IntTemp_Avg;
	float f_IntTemp;

	unsigned int counter;		//Cuántas muestras promedio
} ADCReadout;

//float f_VDAC;
int i_VDAC;

#define AVG			8000
#define AVG_Calc	8

uint16_t ADCBuffer[7];

//****************************************************************************************
//****************************************************************************************


//------------------------------------------------------------------------------
// Variables de Bus 595 (Front Panel)
union {
	unsigned char uc_B595[4];

	struct {
		//uc_B595[0]
		union{
			unsigned char LEDs:8;
			struct {
				unsigned char OutOn:1;
				unsigned char R3:1;
				unsigned char R2:1;
				unsigned char R4:1;
				unsigned char R1:1;
				unsigned char CV:1;
				unsigned char CC:1;
				unsigned char Lock:1;
			};
		};

		//uc_B595[1]
		unsigned char Current:8;

		//uc_B595[2]
		unsigned char Voltage:8;

		//uc_B595[3]
		union{
			unsigned char Digits:8;
			struct {
				unsigned char V1:1;
				unsigned char V4:1;
				unsigned char V3:1;
				unsigned char C4:1;
				unsigned char C3:1;
				unsigned char C2:1;
				unsigned char C1:1;
				unsigned char V2:1;
			};
		};
	};
} FrontPanel;
//****************************************************************************************
//****************************************************************************************


//------------------------------------------------------------------------------
// Variables de Calibración MultiPunto

struct {
	uint8_t flg_newData:1;
	uint8_t MoreFlags:7;

	uint32_t newValue;
} MultiPointCal;



//****************************************************************************************
//****************************************************************************************


//Variables de cálculo de Regresión Lineal
/*
Cálculo de regresión lineal de los valores de Vfb (V feedback), Vfb2 (V feedback 2, cable independiente a la salida),
VSet (Valor leído de seteo), y VDAC (Valor seteado en el DAC).

Los cálculos están basados en las siguientes ecuaciones de regresión Lineal
	SumY = a * SumX + b * n

	SumXY = a * SumX2 + b * SumX

Donde:
	SumY = Sumatoria de valores de Y (Valor de tensión leído a la salida)
	SumX = Sumatoria de valores de X (Valor de X, que se calcula simultáneamente para las variables descritas al inicio)
	SumXY = Sumatoria de la multiplicación X * Y
	SumX2 = Sumatoria de la multiplicación X * X, o X^2
	n = Cantidad de datos utilizados para el cálculo

	a = Pendiente de la recta de conversión
	b = Ordenada al origen de la recta de conversión

http://www.uca.edu.sv/matematica/upload_w/file/REGRESION%20SIMPLE%20Y%20MULTIPLE.pdf

a = (SumY - n * b) / SumX

b= (SumXY - (SumY * SumX2 / SumX)) / (SumX - (n * SumX2 / SumX))

 */

float f_sumY, f_sumY_DAC;			//Valores de V
float f_sumX_fb, f_sumXY_fb, f_sumX2_fb, f_DataX_fb;		//Variables de cálculo de Vfb
float f_sumX_fb2, f_sumXY_fb2, f_sumX2_fb2, f_DataX_fb2;		//Variables de cálculo de Vfb2
float f_sumX_Set, f_sumXY_Set, f_sumX2_Set, f_DataX_Set;		//Variables de cálculo de VSet
float f_sumX_DAC, f_sumXY_DAC, f_sumX2_DAC, f_DataX_DAC;		//Variables de cálculo de DAC
float f_CalAux, f_CalAux2; //Variables auxiliares de cálculo

unsigned char uc_NPoints;

unsigned char tx_counter;

unsigned char flg_calibrate, flg_newData, flg_CalEnd, flg_debug;

float f_newData;
//***************************************EEpromCheck Variables
struct {
	union {
		uint8_t a[2];
		uint16_t A_16;
	};

	union {
		uint8_t b[2];
		uint16_t B_16;
	};

	union {
		uint8_t c[2];
		uint16_t C_16;
	};

	union {
		uint8_t d[2];
		uint16_t D_16;
	};
} EEPROM_Check;


#define CHECK_A 0
#define CHECK_B 1
#define CHECK_C	2
#define CHECK_D	3
/* Virtual address defined by the user: 0xFFFF value is prohibited */
#ifdef __EEPROM_H
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x0011, 0x0022, 0x0033, 0x0044, 0x0055, 0x0066, 0x0077, 0x0088, 0x0099,
		0x00AA, 0x00BB, 0x00CC, 0x00DD, 0x00EE, 0x00FF, 0x1100, 0x1111, 0x1122, 0x1133, 0x1144 };
//uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint16_t VarValue = 0;
#endif


//****************************************************************************************

//***************************************Conversion variables

#define KFB_A	4
#define KFB_B	5
#define KFB2_A	6
#define KFB2_B	7
#define KSET_A	8
#define KSET_B	9
#define KDAC_A	10
#define KDAC_B	11
#define CFB_A	12
#define CFB_B	13
#define CFB2_A	14
#define CFB2_B	15
#define CSET_A	16
#define CSET_B	17
#define CDAC_A	18
#define CDAC_B	19


struct {
	union {
		float K_Vfb;
		int16_t i_kVFb[2];
	};

	union {
		float K_Vfb2;
		int16_t i_kVFb2[2];
	};

	union {
		float K_VSet;
		int16_t i_kVSet[2];
	};

	union {
		float K_Ifb;
		int16_t i_kIFb[2];
	};

	union {
		float K_ISet;
		int16_t i_kISet[2];
	};

	union {
		float K_DAC;
		int16_t i_kDAC[2];
	};

	union {
		float C_Vfb;
		int16_t i_cVFb[2];
	};

	union {
		float C_Vfb2;
		int16_t i_cVFb2[2];
	};

	union {
		float C_VSet;
		int16_t i_cVSet[2];
	};

	union {
		float C_Ifb;
		int16_t i_cIFb[2];
	};

	union {
		float C_ISet;
		int16_t i_cISet[2];
	};

	union {
		float C_DAC;
		int16_t i_cDAC[2];
	};

} Convert;

//****************************************************************************************

float f_Vfb, f_Vfb2, f_VSet, f_VDAC;

//****************************************************************************************
//****************************************************************************************
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM15_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void Refresh_digits(void);
char Decode_7seg(char num);
void Process_number(int num);
void CalV_MultiPoints(void);
void ProcessMsj(void);

uint8_t CheckEEPROM(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Refresh_digits(void) {

	/*for(uc_aux=0;uc_aux<4;uc_aux++){
		HAL_SPI_Transmit(&hspi1,&uc_595bus[uc_aux],1,1);
	}*/
	HAL_SPI_Transmit(&hspi1,&FrontPanel.uc_B595[0],4,1);
	//HAL_Delay(1);
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	HAL_GPIO_WritePin(Refresh_GPIO_Port,Refresh_Pin,1);
	//HAL_Delay(1);

	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");
	asm("NOP");

	HAL_GPIO_WritePin(Refresh_GPIO_Port,Refresh_Pin,0);

}

void Process_number(int num) {
	Process.mil=num/1000;
	Process.cen=num/100;
	Process.dec=num/10;
	Process.uni=num;
	Process.uni-=Process.dec*10;
	Process.dec-=Process.cen*10;
	Process.cen-=Process.mil*10;

	Process.mil=Decode_7seg(Process.mil);
	Process.cen=Decode_7seg(Process.cen);
	Process.dec=Decode_7seg(Process.dec);
	Process.uni=Decode_7seg(Process.uni);
}

char Decode_7seg(char num) {

	char c_return=0;

	/*if(num==0) c_return=cero;
	else if(num==1) c_return=uno;
	else if(num==2) c_return=dos;
	else if(num==3) c_return=tres;
	else if(num==4) c_return=cuatro;
	else if(num==5) c_return=cinco;
	else if(num==6) c_return=seis;
	else if(num==7) c_return=siete;
	else if(num==8) c_return=ocho;
	else if(num==9) c_return=nueve;
	else c_return=0;*/
	switch (num) {
	case 0:
		c_return=cero;
		break;
	case 1:
		c_return=uno;
		break;
	case 2:
		c_return=dos;
		break;
	case 3:
		c_return=tres;
		break;
	case 4:
		c_return=cuatro;
		break;
	case 5:
		c_return=cinco;
		break;
	case 6:
		c_return=seis;
		break;
	case 7:
		c_return=siete;
		break;
	case 8:
		c_return=ocho;
		break;
	case 9:
		c_return=nueve;
		break;
	default:
		c_return=0;
		break;
	}

	return c_return;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  __disable_irq();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_USB_DEVICE_Init();
  MX_TIM15_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */


  CheckEEPROM();

  FrontPanel.uc_B595[0]=0;
  FrontPanel.uc_B595[1]=0;
  FrontPanel.uc_B595[2]=0;
  FrontPanel.uc_B595[3]=0;

  FrontPanel.Voltage=cinco;
  FrontPanel.V2=1;
  FrontPanel.OutOn=0;
  FrontPanel.R1=1;
  FrontPanel.R2=1;
  FrontPanel.R3=1;
  FrontPanel.R4=1;

  Refresh_digits();

  QuadEnc.multiplier=1;
  TIM1->CNT=100;
  TIM17->CCR1=100;
  TIM16->CCR1=512;
  TIM2->CCR1=0;
  TIM2->CCR4=0;
  V_Set=0;
  I_Set=0;
  QuadEnc.Vabsolute=0;
  QuadEnc.Iabsolute=0;
  QuadEnc.state=EncoderVoltage;

  BuzzerPWM=0xFF;

  HAL_TIM_Encoder_Start_IT(&htim1,htim1.Channel);
  HAL_TIM_PWM_Start(&htim17,htim17.Channel);
  HAL_TIM_PWM_Start(&htim16,htim16.Channel);
  HAL_TIM_PWM_Start(&htim2,htim2.Channel);
  HAL_TIM_PWM_Start(&htim14,htim14.Channel);

  HAL_UART_Receive_IT(&huart2,&UART_RX.data,1);

  HAL_ADC_Start_DMA(&hadc,(uint32_t*)ADCBuffer,7);
  /*for(flg_received=0;flg_received<20;flg_received++){
	  RxMsj[flg_received]=0;
  }*/
  //flg_received=0;
  UART_RX.flg_newMsj=0;

  flg_debug=1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  __enable_irq();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(UART_RX.flg_data){

		  //HAL_UART_Transmit(&huart2,UART_RX.Message,UART_RX.count,10);

		  ProcessMsj();

		  for(UART_RX.count=0;UART_RX.count<50;UART_RX.count++) {
			  UART_RX.Message[UART_RX.count]=0;
		  }
		  UART_RX.count=0;
		  UART_RX.flg_data=0;
	  }

	  if(flg_tick) {			//100ms
		  flg_tick=0;

		  ui_contadorDisplay++;

		  if(ui_contadorDisplay%10){					//Blink what is for edit
			  if(QuadEnc.state==EncoderVoltage) {			//Blink edit VoltSet
				  FrontPanel.CV=1;
			  } else if(QuadEnc.state==EncoderCurrent) {		//Blink edit CurrentSet
				  FrontPanel.CC=1;
			  }
		  } else {									//Blink - Off part
			  FrontPanel.CV=0;
			  FrontPanel.CC=0;
		  }

		  if(ADCReadout.flg_newdata) {
			  ADCReadout.VFb_Avg*=Convert.K_Vfb;
			  ADCReadout.VFb_Avg+=Convert.C_Vfb;

			  ADCReadout.VFb2_Avg*=Convert.K_Vfb2;
			  ADCReadout.VFb2_Avg+=Convert.C_Vfb2;

			  ADCReadout.VSet_Avg*=Convert.K_VSet;
			  ADCReadout.VSet_Avg+=Convert.C_VSet;

			  ADCReadout.IFb_Avg*=Convert.K_Ifb;
			  ADCReadout.IFb_Avg+=Convert.C_Ifb;

			  ADCReadout.ISet_Avg*=Convert.K_Ifb;
			  ADCReadout.ISet_Avg+=Convert.C_Ifb;

			  f_VDAC=ADCReadout.VFb_Avg*Convert.K_DAC;
			  f_VDAC+=Convert.C_DAC;

			  ADCReadout.flg_newdata=0;

			  intShow = ADCReadout.IFb_Avg/100;
			  i_VDAC = ADCReadout.VFb2_Avg/100;

			  if(i_VDAC>10000) {
				  i_VDAC/=10;
				  flg_lessThan10=0;
			  } else {
				  flg_lessThan10=1;
			  }

			  if(intShow>10000) {
				  intShow/=10;
				  flg_IlessThan10=0;
			  } else {
				  flg_IlessThan10=1;
			  }
		  }

		  V_Set=QuadEnc.Vabsolute;
		  I_Set=QuadEnc.Iabsolute;

		  Process_number(intShow);
		  DispC.completo=Process.completo;

		  Process_number(i_VDAC);
		  DispV.completo=Process.completo;

		  /*if(RxBuf[0]=='a') {
			  sprintf(RxMsj,"Received: ");
			  strcat(RxMsj,RxBuf);
			  length=strlen(RxMsj);
			  //CDC_Transmit_FS(Buffer,length);
			  HAL_UART_Transmit(&huart2,RxMsj,length,10);
			  RxBuf[0]=0;
		  }*/

		  if(flg_debug){
			  flg_debug=0;
			  length=sprintf(Buffer,"Vfb:%d - Vfb2:%d - Vset:%d - V_DAC:0x%X\n\r",ADCReadout.VFb_Avg,ADCReadout.VFb2_Avg,ADCReadout.VSet_Avg, V_Set);
			  length+=sprintf(RxBuf,"Ifb:%d - ISet:%d - I_DAC:0x%X\n\r",ADCReadout.IFb_Avg, ADCReadout.ISet_Avg, I_Set);
			  strcat(Buffer,RxBuf);
			  //length=strlen(Buffer);
			  //CDC_Transmit_FS(Buffer,length);
			  HAL_UART_Transmit_DMA(&huart2,Buffer,length);
		  }

		  if(flg_calibrate) {
			  CalV_MultiPoints();
		  }
		  //FrontPanel.Voltage=uc_uni;
		  //FrontPanel.Current=uc_dec;
		  //if(!HAL_GPIO_ReadPin(Bt_M1_GPIO_Port,Bt_M1_Pin)) show=0;
		  if(!HAL_GPIO_ReadPin(Bt_M2_GPIO_Port,Bt_M2_Pin)) show=1;
		  if(!HAL_GPIO_ReadPin(Bt_M3_GPIO_Port,Bt_M3_Pin)) show=2;
		  if(!HAL_GPIO_ReadPin(Bt_Lock_GPIO_Port,Bt_Lock_Pin)) show=3;
		  if(!HAL_GPIO_ReadPin(Bt_CC_GPIO_Port,Bt_CC_Pin)) show=4;
		  if(!HAL_GPIO_ReadPin(Bt_CV_GPIO_Port,Bt_CV_Pin)) show=5;

		  if(!HAL_GPIO_ReadPin(Rt_Sw_GPIO_Port,Rt_Sw_Pin)) QuadEnc.Valor=0;

		  /*if(!HAL_GPIO_ReadPin(Bt_OutOn_GPIO_Port,Bt_OutOn_Pin)) {
			  if(FrontPanel.OutOn) {
				  FrontPanel.OutOn=0;
				  HAL_GPIO_WritePin(RelayOut_GPIO_Port,RelayOut_Pin,0);
			  } else {
				  FrontPanel.OutOn=1;
				  HAL_GPIO_WritePin(RelayOut_GPIO_Port,RelayOut_Pin,1);
			  }
		  }*/

	  }

	  /*if(flg_received) {			//Recibió msj por USB_UART
		  flg_received=0;
		  //strcpy(RxBuf,)
		  length=strlen(RxBuf);
		  data=RxBuf[0];

		  e_timeout=0;
		  while(CDC_Transmit_FS(RxBuf,1)!=USBD_OK) {
			  e_timeout++;
			  if(e_timeout>100) break;
		  }

		  if(RxBuf[0]>13 && msjlength<50) {

			  if(FrontPanel.Lock) FrontPanel.Lock=0;
			  else FrontPanel.Lock=1;
			  strcat(RxMsj,&RxBuf[0]);
			  msjlength++;

		  } else {
			  if(FrontPanel.CC) FrontPanel.CC=0;
			  else FrontPanel.CC=1;

			  e_timeout=0;
			  while(CDC_Transmit_FS("\n\r",2)!=USBD_OK) {
				  e_timeout++;
				  if(e_timeout>100) break;
			  }

			  ProcessMsj();
		  }
	  }*/

	  if(flg_RD) {				//3ms
		  flg_RD=0;
		  uc_Digit++;
		  switch(uc_Digit){
		  case 1:
			  FrontPanel.Digits=0;
			  FrontPanel.V4=1;
			  FrontPanel.C4=1;
			  FrontPanel.Voltage=DispV.uni;
			  FrontPanel.Current=DispC.uni;
			  break;
		  case 2:
			  FrontPanel.Digits=0;
			  FrontPanel.V3=1;
			  FrontPanel.C3=1;
			  FrontPanel.Voltage=DispV.dec;
			  FrontPanel.Current=DispC.dec;
			  break;
		  case 3:
			  FrontPanel.Digits=0;
			  FrontPanel.V2=1;
			  FrontPanel.C2=1;
			  FrontPanel.Voltage=DispV.cen;
			  FrontPanel.Current=DispC.cen;
			  if(!flg_lessThan10) FrontPanel.Voltage|=0x01;			//Set dot
			  if(!flg_IlessThan10) FrontPanel.Current|=0x01;			//Set dot
			  //else FrontPanel.Voltage&=0xFE;						//clear dot
			  break;
		  case 4:
			  FrontPanel.Digits=0;
			  FrontPanel.V1=1;
			  FrontPanel.C1=1;
			  FrontPanel.Voltage=DispV.mil;
			  FrontPanel.Current=DispC.mil;
			  if(flg_lessThan10) FrontPanel.Voltage|=0x01;			//Set dot
			  if(flg_IlessThan10) FrontPanel.Current|=0x01;			//Set dot
			  //else FrontPanel.Voltage&=0xFE;						//clear dot
			  uc_Digit=0;
			  break;
		  }

		  Refresh_digits();
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
  RCC_CRSInitTypeDef RCC_CRSInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the SYSCFG APB clock 
  */
  __HAL_RCC_CRS_CLK_ENABLE();
  /** Configures CRS 
  */
  RCC_CRSInitStruct.Prescaler = RCC_CRS_SYNC_DIV1;
  RCC_CRSInitStruct.Source = RCC_CRS_SYNC_SOURCE_USB;
  RCC_CRSInitStruct.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  RCC_CRSInitStruct.ReloadValue = __HAL_RCC_CRS_RELOADVALUE_CALCULATE(48000000,1000);
  RCC_CRSInitStruct.ErrorLimitValue = 34;
  RCC_CRSInitStruct.HSI48CalibrationValue = 32;

  HAL_RCCEx_CRSConfig(&RCC_CRSInitStruct);
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_7;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted. 
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 32768;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 1024;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 1024;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 1024;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 128;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */
  HAL_TIM_MspPostInit(&htim17);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel4_5_6_7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_5_6_7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_5_6_7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RelayOut_Pin|RelayB_Pin|RelayA_Pin|Refresh_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Bt_M3_Pin Bt_M2_Pin Bt_M1_Pin */
  GPIO_InitStruct.Pin = Bt_M3_Pin|Bt_M2_Pin|Bt_M1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Bt_Lock_Pin */
  GPIO_InitStruct.Pin = Bt_Lock_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Bt_Lock_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RelayOut_Pin RelayB_Pin RelayA_Pin */
  GPIO_InitStruct.Pin = RelayOut_Pin|RelayB_Pin|RelayA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Refresh_Pin */
  GPIO_InitStruct.Pin = Refresh_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Refresh_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Rt_Sw_Pin */
  GPIO_InitStruct.Pin = Rt_Sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Rt_Sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Bt_CC_Pin */
  GPIO_InitStruct.Pin = Bt_CC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Bt_CC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Bt_CV_Pin Bt_OutputOn_Pin */
  GPIO_InitStruct.Pin = Bt_CV_Pin|Bt_OutputOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {				//External Interrupt on Pin
	if(GPIO_Pin == GPIO_PIN_7) {									//PB7 falling edge interrupt (Output On Button)
		if(!flg_TS) {
			if(FrontPanel.OutOn) {
				FrontPanel.OutOn=0;
				HAL_GPIO_WritePin(RelayOut_GPIO_Port,RelayOut_Pin,0);
			} else {
				FrontPanel.OutOn=1;
				HAL_GPIO_WritePin(RelayOut_GPIO_Port,RelayOut_Pin,1);
			}
			flg_TS=4;
		}
	}

	if(GPIO_Pin == GPIO_PIN_6) {					//CV button
		if(!flg_TS) {
			if(QuadEnc.state!=EncoderVoltage) {		//Wasn't in CV encoder mode
				QuadEnc.CalcCount=QuadEnc.Vcount;
				QuadEnc.CalcCountAnt=QuadEnc.Vcount;
				ENCODER=QuadEnc.Vcount;
				QuadEnc.state=EncoderVoltage;
			}
			flg_TS=4;
		}
	}

	if(GPIO_Pin == GPIO_PIN_15) {					//CC button
		if(!flg_TS) {
			if(QuadEnc.state!=EncoderCurrent) {		//Wasn't in CV encoder mode
				QuadEnc.CalcCount=QuadEnc.Icount;
				QuadEnc.CalcCountAnt=QuadEnc.Icount;
				ENCODER=QuadEnc.Icount;
				QuadEnc.state=EncoderCurrent;
			}
			flg_TS=4;
		}
	}
}

void HAL_IncTick(void){

	uc_timer++;
	if(uc_timer>99){
		flg_tick=1;				//flg_100ms
		if(flg_TS) flg_TS--;
		uc_timer=0;
		//HAL_GPIO_TogglePin(Refresh_GPIO_Port,Refresh_Pin);
		tx_counter++;
		if(tx_counter>9) {
			tx_counter=0;
			flg_debug=1;
		}
	}


	QuadEnc.timeout++;				//timeout para reducir multiplier
	if(QuadEnc.timeout>500) {		//cada medio segundo actualiza el multiplicador
		QuadEnc.timeout=0;
		QuadEnc.vel=0;				//Resetea contador de velocidad
		if(QuadEnc.multiplier>10) QuadEnc.multiplier=10;
		else QuadEnc.multiplier=1;

	}


	uc_RD++;
	if(uc_RD>5) {
		flg_RD=1;				//flg_3ms
		uc_RD=0;

		QuadEnc.CalcCount=ENCODER;		//Lee valor de encoder

		if(QuadEnc.CalcCount!=QuadEnc.CalcCountAnt) {			//Si es distinto al anterior, se giró el encoder
			QuadEnc.aux=QuadEnc.CalcCount-QuadEnc.CalcCountAnt;			//diferencia de giro

			if(QuadEnc.state==EncoderVoltage) {
				QuadEnc.Vabsolute+=QuadEnc.aux*QuadEnc.multiplier;		//Incrementa el valor de la cuenta afectado por el multiplicador
				QuadEnc.Vcount=QuadEnc.CalcCount;
			} else if(QuadEnc.state==EncoderCurrent) {
				QuadEnc.Iabsolute+=QuadEnc.aux*QuadEnc.multiplier;
				QuadEnc.Icount=QuadEnc.CalcCount;
			} else {
				QuadEnc.state=EncoderVoltage;
			}
			//if(QuadEnc.Valor>9999) QuadEnc.Valor=9999;				//Limita el valor máximo
			//if (QuadEnc.Valor<0) QuadEnc.Valor=0;				//Limita el valor mínimo

			QuadEnc.vel++;								//Incrementa contador de velocidad
			QuadEnc.timeout=0;							//Y resetea timeout

			QuadEnc.CalcCountAnt=QuadEnc.CalcCount;			//Actualiza valor anterior
			if(QuadEnc.vel>50) {
				QuadEnc.vel=0;
				if(QuadEnc.multiplier<10) QuadEnc.multiplier=10;
				else if(QuadEnc.multiplier<100) QuadEnc.multiplier=100;
				else if(QuadEnc.multiplier<1000) QuadEnc.multiplier=1000;
			}
		}
	}

};


//***********************************ADC Conversion Complete
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {

	ADCReadout.Temp+=ADCBuffer[0];
	ADCReadout.VFb2+=ADCBuffer[1];
	ADCReadout.IFb+=ADCBuffer[2];
	ADCReadout.ISet+=ADCBuffer[3];
	ADCReadout.VSet+=ADCBuffer[4];
	ADCReadout.VFb+=ADCBuffer[5];
	ADCReadout.IntTemp+=ADCBuffer[6];

	ADCReadout.counter++;

	if(ADCReadout.counter==AVG) {
		ADCReadout.VFb2_Avg=ADCReadout.VFb2 / AVG_Calc;				//Measure in mV
		ADCReadout.IFb_Avg=ADCReadout.IFb/AVG_Calc;					//Measure in mA
		ADCReadout.VFb_Avg=ADCReadout.VFb/AVG_Calc;
		ADCReadout.VSet_Avg=ADCReadout.VSet/AVG_Calc;
		ADCReadout.ISet_Avg=ADCReadout.ISet/AVG_Calc;

		ADCReadout.VFb2=0;
		ADCReadout.IFb=0;
		ADCReadout.VFb=0;
		ADCReadout.VSet=0;
		ADCReadout.ISet=0;
		ADCReadout.counter=0;

		ADCReadout.flg_newdata=1;

	}

};



void CalV_MultiPoints (void) {
    if(MultiPointCal.flg_newData) {
    	MultiPointCal.flg_newData=0;

        uc_NPoints++;               //Cantidad de valores recibidos hasta el momento

        //SumY
        f_sumY+=MultiPointCal.newValue;         				//Sumatoria Y
        f_sumY_DAC+=V_Set;						//Para el caso del DAC, el valor Y es el valor de DAC seteado

        //SumX
        f_sumX_fb+=ADCReadout.VFb_Avg;          //Sumatoria Vfb
        f_sumX_fb2+=ADCReadout.VFb2_Avg;        //Sumatoria Vfb2
        f_sumX_Set+=ADCReadout.VSet_Avg;        //Sumatoria VSet
        f_sumX_DAC+=MultiPointCal.newValue;              	//Sumatoria VDAC

        //SumXY
        //Vfb
        f_CalAux=MultiPointCal.newValue*ADCReadout.VFb_Avg;
        f_sumXY_fb+=f_CalAux;
        //Vfb2
		f_CalAux=MultiPointCal.newValue*ADCReadout.VFb2_Avg;
		f_sumXY_fb2+=f_CalAux;
		//VSet
		f_CalAux=f_newData*ADCReadout.VSet_Avg;
		f_sumXY_Set+=f_CalAux;
		//VDAC
		f_CalAux=MultiPointCal.newValue*V_Set;
		f_sumXY_DAC+=f_CalAux;

		//Sumatoria X^2
		//Vfb
		f_CalAux=ADCReadout.VFb_Avg*ADCReadout.VFb_Avg;
		f_sumX2_fb+=f_CalAux;
		//Vfb2
		f_CalAux=ADCReadout.VFb2_Avg*ADCReadout.VFb2_Avg;
		f_sumX2_fb2+=f_CalAux;
		//VSet
		f_CalAux=ADCReadout.VSet_Avg*ADCReadout.VSet_Avg;
		f_sumX2_Set+=f_CalAux;
		//VDAC
		f_CalAux=MultiPointCal.newValue*MultiPointCal.newValue;
		f_sumX2_DAC+=f_CalAux;


		//length=sprintf(Buffer,"Vfb:%ld - Vfb2:%ld - Vset:%ld - V_DAC:%ld\n\r",(long) ADCReadout.f_Vfb,(long) ADCReadout.f_Vfb2,(long) ADCReadout.f_VSet,(long) f_VDAC);
		length=sprintf(Buffer,"New Data: %d\n\r",MultiPointCal.newValue);
		HAL_UART_Transmit_DMA(&huart2,Buffer,length);
		//CDC_Transmit_FS(Buffer,length);
    }

    if(flg_CalEnd) {

    	//Cálculo de Pendiente (K) y de ordenada al origen (C)
    	//Vfb
        if(f_sumX_fb) {                        //C�lculo por regresion lineal
            f_CalAux=uc_NPoints * f_sumX2_fb;
            f_CalAux/=f_sumX_fb;
            f_CalAux2=f_sumX_fb-f_CalAux;
            f_CalAux=f_sumY*f_sumX2_fb;
            f_CalAux/=f_sumX_fb;
            Convert.C_Vfb=f_sumXY_fb-f_CalAux;
            if(f_CalAux2) Convert.C_Vfb/=f_CalAux2;
            else Convert.C_Vfb=0;

            f_CalAux=uc_NPoints*Convert.C_Vfb;
            f_CalAux2=f_sumY-f_CalAux;
            Convert.K_Vfb=f_CalAux2/f_sumX_fb;
        } else {
            Convert.K_Vfb=1;
            Convert.C_Vfb=0;
        }

        //Vfb2
		if(f_sumX_fb2) {                        //C�lculo por regresion lineal
			f_CalAux=uc_NPoints * f_sumX2_fb2;
			f_CalAux/=f_sumX_fb2;
			f_CalAux2=f_sumX_fb2-f_CalAux;
			f_CalAux=f_sumY*f_sumX2_fb2;
			f_CalAux/=f_sumX_fb2;
			Convert.C_Vfb2=f_sumXY_fb2-f_CalAux;
			if(f_CalAux2) Convert.C_Vfb2/=f_CalAux2;
			else Convert.C_Vfb2=0;

			f_CalAux=uc_NPoints*Convert.C_Vfb2;
			f_CalAux2=f_sumY-f_CalAux;
			Convert.K_Vfb2=f_CalAux2/f_sumX_fb2;
		} else {
			Convert.K_Vfb2=1;
			Convert.C_Vfb2=0;
		}

		//VSet
		if(f_sumX_Set) {                        //C�lculo por regresion lineal
			f_CalAux=uc_NPoints * f_sumX2_Set;
			f_CalAux/=f_sumX_Set;
			f_CalAux2=f_sumX_Set-f_CalAux;
			f_CalAux=f_sumY*f_sumX2_Set;
			f_CalAux/=f_sumX_Set;
			Convert.C_VSet=f_sumXY_Set-f_CalAux;
			if(f_CalAux2) Convert.C_VSet/=f_CalAux2;
			else Convert.C_VSet=0;

			f_CalAux=uc_NPoints*Convert.C_VSet;
			f_CalAux2=f_sumY-f_CalAux;
			Convert.K_VSet=f_CalAux2/f_sumX_Set;
		} else {
			Convert.K_VSet=1;
			Convert.C_VSet=0;
		}

		//VDAC
		if(f_sumX_DAC) {                        //C�lculo por regresion lineal
			f_CalAux=uc_NPoints * f_sumX2_DAC;
			f_CalAux/=f_sumX_DAC;
			f_CalAux2=f_sumX_DAC-f_CalAux;
			f_CalAux=f_sumY_DAC*f_sumX2_DAC;
			f_CalAux/=f_sumX_DAC;
			Convert.C_DAC=f_sumXY_DAC-f_CalAux;
			if(f_CalAux2) Convert.C_DAC/=f_CalAux2;
			else Convert.C_DAC=0;

			f_CalAux=uc_NPoints*Convert.C_DAC;
			f_CalAux2=f_sumY_DAC-f_CalAux;
			Convert.K_DAC=f_CalAux2/f_sumX_DAC;
		} else {
			Convert.K_DAC=1;
			Convert.C_DAC=0;
		}

		length=sprintf(Buffer,"K_Vfb: %d - C_fb:%d\n\rK_fb2: %d - C_fb2:%d\n\r", (int) Convert.K_Vfb * 10000, (int) Convert.C_Vfb *100, (int) Convert.K_Vfb2 *10000, (int) Convert.C_Vfb2*100);
		length+=sprintf(RxBuf,"K_Set: %d - C_VSet:%d\n\rK_DAC: %d - C_DAC:%d\n\r",(int) Convert.K_VSet*10000,(int) Convert.C_VSet*100, (int) Convert.K_DAC*10000,(int) Convert.C_DAC*100);
		strcat(Buffer,RxBuf);
		HAL_UART_Transmit_DMA(&huart2,Buffer,length);

		flg_calibrate=0;
    }
}

void ProcessMsj(void) {

	char flg_parameter;
	char RxLength;
	char *ui_Pointer;//*uc_RX_Pointer
	char st_aux[50], st_aux2[20];
	//extern unsigned char RxMsj[50];


	//RxLength=strlen(RxMsj);
	if(UART_RX.Message[0]=='a' && UART_RX.Message[UART_RX.count-1]=='+') {

		strncpy(st_aux,&UART_RX.Message[1],UART_RX.count-2);

		ui_Pointer=strchr(st_aux,',');
		if(*ui_Pointer){              //Hay una coma en el mensaje, hay par�metros
			flg_parameter=1;
		} else {                        //no hay coma, no hay par�metros, simplemente pregunto por alg�n valor
			flg_parameter=0;
		}

//---------------------------Analiza qué comando recibió------------------------
		if(strstr(st_aux,"cal1")){
			flg_calibrate=1;
			e_timeout=0;
			HAL_UART_Transmit(&huart2,"Commence Calibration\n\r",22,10);
			/*while(CDC_Transmit_FS("Commence Calibration\n\r",22)!=USBD_OK){
				e_timeout++;
				if(e_timeout>100) break;
			}*/
			f_sumX_DAC=0;
			f_sumX_Set=0;
			f_sumX_fb=0;
			f_sumX_fb2=0;

			f_sumX2_DAC=0;
			f_sumX2_Set=0;
			f_sumX2_fb=0;
			f_sumX2_fb2=0;

			f_sumXY_DAC=0;
			f_sumXY_Set=0;
			f_sumXY_fb=0;
			f_sumXY_fb2=0;

			f_sumY=0;
			f_sumY_DAC=0;

			uc_NPoints=0;
		}
		//****************************************Tension de calibraci�n
		else if(strstr(st_aux,"vcal")){
			if(flg_parameter){
				ui_Pointer++;
				strcpy(st_aux2,ui_Pointer);

				RxLength=strlen(st_aux2);
				RxLength--;

				//f_newData=strtof(st_aux2,NULL) /100.0;//&st_aux2[RxLength]);
				MultiPointCal.newValue=atol(st_aux2)*100;
				//f_newData*=10;
				length=sprintf(st_aux,"N: %d\n\r",(int) MultiPointCal.newValue);
				HAL_UART_Transmit(&huart2,st_aux,length,10);
				e_timeout=0;
				/*while(CDC_Transmit_FS(st_aux,RxLength)!=USBD_OK){
					e_timeout++;
					if(e_timeout>100) break;
				}*/
				//flg_newData=1;
				MultiPointCal.flg_newData=1;
			}
		}
		//****************************************Fin_CalMultiPunto
		else if(strstr(st_aux,"CalEnd")){
			flg_CalEnd=1;
		}
		//****************************************Fin_CalMultiPunto
		else if(strstr(st_aux,"debug")){
			if(flg_debug) flg_debug=0;
			else flg_debug=1;
		}

		else {
			HAL_UART_Transmit(&huart2,"Comando erroneo\n\r",17,10);
		}
	}

	/*for(flg_parameter=0;flg_parameter<50;flg_parameter++){
		RxMsj[flg_parameter]=0;
	}

	msjlength=0;*/

}


uint8_t CheckEEPROM(void){

#ifdef __EEPROM_H
	uint8_t EEPROM_Checked=0;
	uint16_t ReadStatus=0;
	Convert.K_Ifb=1;
	HAL_FLASH_Unlock();

	ReadStatus=EE_Init();
	length=sprintf(Buffer,"INIT: 0x%X\n\r",ReadStatus);
	HAL_UART_Transmit(&huart2,Buffer,length,10);

	ReadStatus=EE_ReadVariable(VirtAddVarTab[CHECK_A],&EEPROM_Check.A_16);
	//length=sprintf(Buffer,"R_Status: 0x%X - A32:0x%X\n\r",ReadStatus,EEPROM_Check.A_16);
	//HAL_UART_Transmit(&huart2,Buffer,length,10);

	ReadStatus=EE_ReadVariable(VirtAddVarTab[CHECK_B],&EEPROM_Check.B_16);
	//length=sprintf(Buffer,"R_Status: 0x%X - B32:0x%X\n\r",ReadStatus,EEPROM_Check.B_16);
	//HAL_UART_Transmit(&huart2,Buffer,length,10);

	ReadStatus=EE_ReadVariable(VirtAddVarTab[CHECK_C],&EEPROM_Check.C_16);
	//length=sprintf(Buffer,"R_Status: 0x%X - C32:0x%X\n\r",ReadStatus,EEPROM_Check.C_16);
	//HAL_UART_Transmit(&huart2,Buffer,length,10);

	ReadStatus=EE_ReadVariable(VirtAddVarTab[CHECK_D],&EEPROM_Check.D_16);
	//length=sprintf(Buffer,"R_Status: 0x%X - D32:0x%X\n\r",ReadStatus,EEPROM_Check.D_16);
	//HAL_UART_Transmit(&huart2,Buffer,length,10);

	ReadStatus=0;
	for(EEPROM_Checked=0;EEPROM_Checked<2;EEPROM_Checked++) {
		if(EEPROM_Check.a[EEPROM_Checked]!=EEPROM_Checked) {
			ReadStatus=1;
			break;
		}
		if(EEPROM_Check.b[EEPROM_Checked]!=EEPROM_Checked+2) {
			ReadStatus=1;
			break;
		}
		if(EEPROM_Check.c[EEPROM_Checked]!=EEPROM_Checked+4) {
			ReadStatus=1;
			break;
		}
		if(EEPROM_Check.d[EEPROM_Checked]!=EEPROM_Checked+6) {
			ReadStatus=1;
			break;
		}
	}

	if(ReadStatus) {			//Error in CheckEEPROM
		for(EEPROM_Checked=0;EEPROM_Checked<2;EEPROM_Checked++) {
			EEPROM_Check.a[EEPROM_Checked]=EEPROM_Checked;
			EEPROM_Check.b[EEPROM_Checked]=EEPROM_Checked+2;
			EEPROM_Check.c[EEPROM_Checked]=EEPROM_Checked+4;
			EEPROM_Check.d[EEPROM_Checked]=EEPROM_Checked+6;
		}
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CHECK_A],EEPROM_Check.A_16);
		length=sprintf(Buffer,"W_Status: 0x%X\n\r",ReadStatus);
		HAL_UART_Transmit(&huart2,Buffer,length,10);

		ReadStatus=EE_WriteVariable(VirtAddVarTab[CHECK_B],EEPROM_Check.B_16);
		length=sprintf(Buffer,"W_Status: 0x%X\n\r",ReadStatus);
		HAL_UART_Transmit(&huart2,Buffer,length,10);

		ReadStatus=EE_WriteVariable(VirtAddVarTab[CHECK_C],EEPROM_Check.C_16);
		length=sprintf(Buffer,"W_Status: 0x%X\n\r",ReadStatus);
		HAL_UART_Transmit(&huart2,Buffer,length,10);

		ReadStatus=EE_WriteVariable(VirtAddVarTab[CHECK_D],EEPROM_Check.D_16);
		length=sprintf(Buffer,"W_Status: 0x%X\n\r",ReadStatus);
		HAL_UART_Transmit(&huart2,Buffer,length,10);


		Convert.K_DAC=1;//6.554155416;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KDAC_A],Convert.i_kDAC[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KDAC_B],Convert.i_kDAC[1]);

		Convert.K_VSet=1;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KSET_A],Convert.i_kVSet[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KSET_B],Convert.i_kVSet[1]);

		Convert.K_Vfb=1;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KFB_A],Convert.i_kVFb[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KFB_B],Convert.i_kVFb[1]);

		Convert.K_Vfb2=1;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KFB2_A],Convert.i_kVFb2[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[KFB2_B],Convert.i_kVFb2[1]);

		Convert.C_DAC=0;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CDAC_A],Convert.i_cDAC[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CDAC_B],Convert.i_cDAC[1]);

		Convert.C_VSet=0;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CSET_A],Convert.i_cVSet[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CSET_B],Convert.i_cVSet[1]);

		Convert.C_Vfb=0;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CFB_A],Convert.i_cVFb[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CFB_B],Convert.i_cVFb[1]);

		Convert.C_Vfb2=0;
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CFB2_A],Convert.i_cVFb2[0]);
		ReadStatus=EE_WriteVariable(VirtAddVarTab[CFB2_B],Convert.i_cVFb2[1]);

		HAL_UART_Transmit(&huart2,"EEPROM ERROR\n\rSet Default Values\n\r",14,10);
	} else {

		ReadStatus=EE_ReadVariable(VirtAddVarTab[KDAC_A],&Convert.i_kDAC[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[KDAC_B],&Convert.i_kDAC[1]);

		ReadStatus=EE_ReadVariable(VirtAddVarTab[KSET_A],&Convert.i_kVSet[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[KSET_B],&Convert.i_kVSet[1]);

		ReadStatus=EE_ReadVariable(VirtAddVarTab[KFB_A],&Convert.i_kVFb[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[KFB_B],&Convert.i_kVFb[1]);

		ReadStatus=EE_ReadVariable(VirtAddVarTab[KFB2_A],&Convert.i_kVFb2[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[KFB2_B],&Convert.i_kVFb2[1]);

		ReadStatus=EE_ReadVariable(VirtAddVarTab[CDAC_A],&Convert.i_cDAC[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[CDAC_B],&Convert.i_cDAC[1]);

		ReadStatus=EE_ReadVariable(VirtAddVarTab[CSET_A],&Convert.i_cVSet[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[CSET_B],&Convert.i_cVSet[1]);

		ReadStatus=EE_ReadVariable(VirtAddVarTab[CFB_A],&Convert.i_cVFb[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[CFB_B],&Convert.i_cVFb[1]);

		ReadStatus=EE_ReadVariable(VirtAddVarTab[CFB2_A],&Convert.i_cVFb2[0]);
		ReadStatus=EE_ReadVariable(VirtAddVarTab[CFB2_B],&Convert.i_cVFb2[1]);

		HAL_UART_Transmit(&huart2,"EEPROM OK\n\r",14,10);
	}

	HAL_FLASH_Lock();

	return EEPROM_Checked;

#endif
}


//*********************************************************UART Receive Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//if(*huart==*huart2) {
	//HAL_UART_Transmit(&huart2,&UART_RX.data,1,10);
		if(UART_RX.count<50 && UART_RX.data>13) {		//Buffer not full, and received data is above "NewLine" ("\n") value
			UART_RX.Message[UART_RX.count]=UART_RX.data;
			UART_RX.count++;
		} else {
			UART_RX.flg_data=1;				//NewMessage received
		}
		HAL_UART_Receive_IT(&huart2,&UART_RX.data,1);
	//}
}

//*************************************************************************************
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
