/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2020 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "SinGenerator.h"
#include <math.h>

/* Overall description
    TIM2 generates PWM output from A0&A1|A0
    TIM3 takts sinGenerator
    TIM4 starts ADC from A2
*/
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int counter=0, teleCounter=0;
uint32_t sinTicksLastZero=0, sinTicksDelay=0;
//_Bool sinSignChange=0;
_Bool filterApFlag=1;//aperiodic filter 1/ts+1, where t=1/5*sinFreq
_Bool teleFlag=0;
_Bool telePhaseFlag=0;//enable or disable telemetry for phase
_Bool sinTicksDelayChanged=0;
int32_t inVal=0, inValPrev=0, inValZero=4096/2;
short unsigned int teleDAIgnore=0;
volatile int sinVal=0;
uint32_t ADC_Buf=0;
_Bool filterDiscrFlag=0;//discrete filter *discFilt state

volatile uint32_t ampValMax=0, ampValMin=4096;// amplitude
uint32_t amp2send=0;
uint32_t ampSendSinTCount=0;

volatile struct DiscrFilt *discFilt;
volatile struct SinGener *sinGen;
_Bool sinValChanged, sinSignChanged;

/*
//temp!!!
int vals[26];
volatile unsigned int indSin=0, indSinLen=26;
volatile unsigned int indTick, indTickMax;
double sinFreqRad=1, sinFreqHz, waveFreqHz;
unsigned int sinAmp;
char indQuad=0;//0:[0 pi/2] 1:[pi/2 pi] 2:[pi 3pi/4] 3:[3pi/4 2pi]
//temp
*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Main_FilterApConfig(void);
void Main_FilterApSetState(_Bool state);
void Main_TeleSetState(_Bool state);
void Main_TelePhaseSetState(_Bool state);
void setPWM(uint16_t);
void Main_FindValZero(unsigned int timeMs);
void Main_delayUS(uint32_t us);
void Main_teleSendValue(void);
void Main_teleSendDA(void);
void Main_teleSendDAValue(void);
void Main_setPWMMode(int mode);

//void Main_FilterKihConfig(void);

//void Main_SinTimerReconfig(void);

_Bool Main_PWMPrescSet(unsigned int prescIn);
_Bool Main_FilterDiscrSetState(_Bool state);
void Main_setCustomSignal(double *arr, unsigned int len);
void Main_setDefaultSignal(void);
void Main_setPulseMin(double percent);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
    /*
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    unsigned long t1 = DWT->CYCCNT;
    */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
    
    TIM2->CCR1=0;

    while(HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK);//- калибровка ацп
    
    //tim2 is used for PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,GPIO_PIN_SET);//polarity of signal indicator
       
    //tim3 is used to generate sin wave
    sinGen=SinGener_new(TIM3, TIM2);
    discFilt=discrFilt_new();
    
    //tim4 is used for adc, and, therefore for filtation at each tick
    
    HAL_TIM_Base_Start_IT(&htim3);
    
    HAL_TIM_Base_Start_IT(&htim4);
    
    /*
    //temp!!
    double step=(acos(-1))/(2*indSinLen);// (pi/2)/len
    sinAmp=TIM2->ARR + 1;
    for(int i=0; i<indSinLen;++i){
        vals[i]=round(sinAmp*sin(step*i));
        vals[i]=abs(vals[i]);
    }
    sinFreqHz=sinFreqRad/(2*acos(-1));// v=w/(2pi)
    waveFreqHz=4*indSinLen*sinFreqHz;//4*len*v
    
    double timerFreqHz=((double)72e6)/
        (
            (TIM3->PSC+1)*
            (TIM3->ARR+1)*
            (TIM3->RCR+1)
        );
    
    indTickMax=round(timerFreqHz/waveFreqHz);
    //temp
    */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
            
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 720-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 64-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 3600-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 36000-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA7 PA8 
                           PA9 PA10 PA13 PA14 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB12 PB13 PB14 
                           PB15 PB3 PB4 PB5 
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Main_Sin_SetFreq(double freqIn){
    sinGener_setSinFreqRad(sinGen, freqIn);
    
    if(filterApFlag){
       Main_FilterApConfig();
    }
    
    teleDAIgnore++;//one more time don't send amp and delay
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
        if(htim->Instance == TIM3){//timer 3 used for setting output voltage level
            _Bool signPositive;
            sinVal=sinGener_tick(sinGen, &sinValChanged, &sinSignChanged, &signPositive);
            
            if(!sinValChanged)return;
            
            if(sinSignChanged){//sign changed
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,signPositive?GPIO_PIN_SET:GPIO_PIN_RESET);//pin a6 indicates sign of signal
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5,signPositive?GPIO_PIN_RESET:GPIO_PIN_SET);//pin a5 is inverted logical value of pin a6

                if(signPositive){
                    sinTicksLastZero=HAL_GetTick();
                
                //amplitude to be sended
                    amp2send=(ampValMax-ampValMin)/2;
                    ampValMax=0;
                    ampValMin=4096;
                }
            }
            return;   
            /*
            //send cur sinVal
            char MesData[5];
                MesData[0]='s';
                for(int i=1;i<4;++i){
                    MesData[4-i]=sinVal%10 + '0';
                    sinVal/=10;
                }
                MesData[4]='\n';
                CDC_Transmit_FS(MesData,5);
            return;
            */
        }
        
        if(htim->Instance == TIM4){//timer 4 used for adc
             ++counter;
            if(counter>1){
                HAL_ADC_Start_DMA(&hadc1, &ADC_Buf, 1);
                HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_9);
                counter=0;  
            }
             return;
        }
        
}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  if(hadc->Instance==ADC1){    
      inVal=ADC_Buf-inValZero;
      
      if(filterApFlag){
        //Простой цифровой фильтр
        inVal=inValPrev + (20*(inVal - inValPrev)>>8);// Na + Nb = 2^k (Nb = 256 — Na); Y(n) = (Na*Y(n-1) + Nb*X(n)) >> k
        //
          
      }
      
      if(filterDiscrFlag){
        inVal=discrFilt_filt(discFilt,inVal);
      }
     
      if(inVal>ampValMax){
          ampValMax=abs(inVal);
      }
      
      if(inVal<ampValMin){
          ampValMin=inVal;
      }
      
      if(((inVal<0)!=(inValPrev<=0))&&inVal>0){//input sign changed to positive
          //sinTicksDelay=sinTCount*sinTiksPerTCount-sinTicksLastZero;
          sinTicksDelay=HAL_GetTick()-sinTicksLastZero;
          sinTicksDelayChanged=1;
          /*
          if(sinTCount > 2000 + ampSendSinTCount){//should be an one fouth part of a cycle, doesn't work yet
            amp2send=ampVal;
            //ampVal=0;
          }*/
      }
      
      inValPrev=inVal;
      ++teleCounter;
      
      if(teleCounter<10){//10^2 Гц частота телеметрии
          return;
      }
      
      if(telePhaseFlag||teleFlag){//if we have sth to send
        HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
        teleCounter=0;
      }
      else {
          return;
      }
            
      if(sinTicksDelayChanged&&telePhaseFlag){
              sinTicksDelayChanged=0;
          if(teleDAIgnore){
            --teleDAIgnore;
            if(teleFlag){
              Main_teleSendDAValue();
              return;
            }
          }
          
          if(teleFlag){
              Main_teleSendDAValue();
              return;
          }
              //transmit phase delay in char format with 'd' prefix and adc val also
              Main_teleSendDA();
              return;
        }
      
        if(teleFlag){
            Main_teleSendValue();
            return;
        }       
  }
}

void Main_teleSendValue(){
    int i;
        uint8_t MesData[7];
        int32_t val=(int) round(inVal);
        MesData[6]='\0';
        MesData[5]='\n';
        MesData[0]=(val>=0)?'+':'-';
        val=abs(val);
        for(i=4;i>=1;--i){
            MesData[i]=val%10 + '0';
            val/=10;
        }
        CDC_Transmit_FS(MesData,strlen(MesData));
}

void Main_teleSendDA(){
    //transmit phase delay and amp in char format with 'd' prefix
    if(amp2send==0) return;

    int i;
    uint8_t MesData[18];
    int32_t val=sinTicksDelay;
    MesData[10]='\n';
    if(val>999999999){
        MesData[0]='D';
    }else{
        MesData[0]='d';
    }

    val=abs(val);
    for(i=9;i>=1;--i){
        MesData[i]=val%10 + '0';
        val/=10;
    }

    val=amp2send;
    MesData[17]='\0';
    MesData[16]='\n';
    if(val>9999){
        MesData[11]='A';
      }else{
        MesData[11]='a';
      }
    for(i=15;i>=12;--i){
        MesData[i]=val%10 + '0';
        val/=10;
    }
    CDC_Transmit_FS(MesData,strlen(MesData));
    
    amp2send=0;
}

void Main_teleSendDAValue(){
//transmit phase delay and amp in char format with 'd' prefix
    if(amp2send==0) {
        Main_teleSendValue();
        return;
    }

    int i;
    uint8_t MesData[24];//18];
    int32_t val=sinTicksDelay;
    MesData[10]='\n';
    if(val>999999999){
        MesData[0]='D';
    }else{
        MesData[0]='d';
    }

    val=abs(val);
    for(i=9;i>=1;--i){
        MesData[i]=val%10 + '0';
        val/=10;
    }

    val=amp2send;
    //MesData[17]='\0';
    MesData[16]='\n';
    if(val>9999){
        MesData[11]='A';
      }else{
        MesData[11]='a';
      }
    for(i=15;i>=12;--i){
        MesData[i]=val%10 + '0';
        val/=10;
    }
    
    val=(int) round(inVal);
    MesData[23]='\0';
    MesData[22]='\n';
    MesData[17]=(val>=0)?'+':'-';
    val=abs(val);
    for(i=21;i>=18;--i){
        MesData[i]=val%10 + '0';
        val/=10;
    }
    CDC_Transmit_FS(MesData,strlen(MesData));
    
    amp2send=0;
}

void Main_FilterApConfig(void){
    return;
}

void Main_FilterApSetState(_Bool state){
    if(state){
        Main_FilterApConfig();
        Main_FilterDiscrSetState(0);
    }
    filterApFlag=state; 
}

_Bool Main_FilterDiscrSetState(_Bool state){
    if(!discrFilt_isReady(discFilt)) return 0;
    
    if(state){
        discrFilt_start(discFilt);
        Main_FilterApSetState(0);
    }
    else{
        discrFilt_stop(discFilt);
    }
    filterDiscrFlag=state;
    return 1;
}
    
void Main_TeleSetState(_Bool state){
    //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,state?GPIO_PIN_SET:GPIO_PIN_RESET);//turns off LED on disabling telemetry
    teleFlag=state;
}

void Main_TelePhaseSetState(_Bool state){
    //HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13,state?GPIO_PIN_SET:GPIO_PIN_RESET);//turns off LED on disabling telemetry
    telePhaseFlag=state;
}

_Bool Main_PWMPrescSet(unsigned int prescIn){
    if(prescIn>0xFFFF){
        return 0;
    }
    if ((HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1)!= HAL_OK)||
        (HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2)!= HAL_OK))
    {
      _Error_Handler(__FILE__, __LINE__);
        return 0;
    }
    
    htim2.Init.Prescaler = prescIn;
    
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
    return 0;
  }
   if ((HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)||
       (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2) != HAL_OK))
  {
    _Error_Handler(__FILE__, __LINE__);
    return 0;
  }   
    return 1;
}

void setPWM(uint16_t pulse){
    switch(sinGener_getModeOut(sinGen)){
        case SG_SINGLEOUT:
            TIM2->CCR1=pulse;
            break;
        case SG_ALTERNATEOUT:
            TIM2->CCR1=pulse;
            TIM2->CCR2=pulse;
        break;
    }
    
    //TIM2->CCR2=sinVal;
    
    //the same
    /*
 HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); // stop generation of pwm
 HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2);
 TIM_OC_InitTypeDef sConfigOC;

 sConfigOC.OCMode = TIM_OCMODE_PWM1;
 sConfigOC.Pulse = pulse; // set the pulse duration
 sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
 sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
 HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
 HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2);    
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // start pwm generation
 HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    */
} 

void Main_FindValZero(unsigned int timeMs){
    HAL_TIM_Base_Stop_IT(&htim3);//sin wave
    HAL_TIM_Base_Stop_IT(&htim4);//telemetry & filtration
    
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop(&hadc1);

    //while(HAL_ADC_GetState(&hadc1)!=HAL_ADC_STATE_RESET);
    
    double mean=0,// cumulative average
    prev_mean, cur;
    unsigned int i=1;
    uint32_t cyclesStop, cyclesCur=0;
    
    setPWM(0);
    
    do{
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1,100);
        cur=HAL_ADC_GetValue(&hadc1);
        prev_mean=mean;
        mean=mean+(cur-mean)/i;
        ++i;
        Main_delayUS(1000);
    }while(abs(mean-prev_mean)>1);
    
    mean=0;
    i=1;
    
    for(int j=0; j<=1;++j){
        cyclesStop=timeMs/2;
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,(j==0)?GPIO_PIN_SET:GPIO_PIN_RESET);//pin a6 indicates sign of signal
        do{
            HAL_ADC_Start(&hadc1);
            HAL_ADC_PollForConversion(&hadc1,100);
            cur=HAL_ADC_GetValue(&hadc1);
            mean=mean+(cur-mean)/i;
            ++i;
            Main_delayUS(1000);
            ++cyclesCur;
        }while(cyclesCur<=cyclesStop);
        HAL_ADC_Stop(&hadc1);
        cyclesCur=0;
        //i1=0;
    }
    inValZero=round(mean);
    
    uint32_t temp=inValZero;
    uint8_t ans[]="in zero value: 4096\n";
    int len=strlen(ans);
    for(int j=len-2;j>(len-6);--j){
        ans[j]=temp%10 + '0';
        temp=temp/10;
    }
    ans[strlen(ans)-1]='\n';
    CDC_Transmit_FS(ans, len);
    
    //MX_ADC1_Init();
    HAL_TIM_Base_Start_IT(&htim3);//sin wave
    HAL_TIM_Base_Start_IT(&htim4);//telemetry & filtration
}

void Main_setModeOutput(int mode){
    sinGener_pause(sinGen);
    sinGener_setModeOut(sinGen, (enum SinGener_ModeOutput)(mode));
    sinGener_resume(sinGen); 
}

void Main_setCustomSignal(double *arr, unsigned int len){
    sinGener_setCusSignal(sinGen, arr, len);
}

void Main_setDefaultSignal(void){
    sinGener_setDefSignal(sinGen);
}

void Main_setPulseMin(double percent){
    sinGener_setPulseMin(sinGen, percent);
}

#pragma push
#pragma O0
void Main_delayUS(uint32_t us) {
	volatile uint32_t counter = 7*us;
	while(counter--);
}
#pragma pop
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
