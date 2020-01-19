/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "MyFunctions.h"
#include "main.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */
/* Define size for the receive and transmit buffer over CDC */
/* It's up to user to redefine and/or remove those define */
#define APP_RX_DATA_SIZE  1000
#define APP_TX_DATA_SIZE  1000
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

  /*******************************************************************************/
  /* Line Coding Structure                                                       */
  /*-----------------------------------------------------------------------------*/
  /* Offset | Field       | Size | Value  | Description                          */
  /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
  /* 4      | bCharFormat |   1  | Number | Stop bits                            */
  /*                                        0 - 1 Stop bit                       */
  /*                                        1 - 1.5 Stop bits                    */
  /*                                        2 - 2 Stop bits                      */
  /* 5      | bParityType |  1   | Number | Parity                               */
  /*                                        0 - None                             */
  /*                                        1 - Odd                              */
  /*                                        2 - Even                             */
  /*                                        3 - Mark                             */
  /*                                        4 - Space                            */
  /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
  /*******************************************************************************/
    case CDC_SET_LINE_CODING:

    break;

    case CDC_GET_LINE_CODING:

    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:
    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will block any OUT packet reception on USB endpoint
  *         untill exiting this function. If you exit this function before transfer
  *         is complete on CDC interface (ie. using DMA controller) it will result
  *         in receiving more data while previous ones are still not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
    bool defFlag=false;
    switch(Buf[0]){
        case 'f':{
            uint32_t len=*Len-1;
            if(!len){
                defFlag=true;
                break;
            }
            double freq=char2double(&Buf[1], &len);
            Main_Sin_SetFreq(freq);
            
            uint8_t ans[]="freq changed\n";
            CDC_Transmit_FS(ans, strlen(ans));
            break;
        }
        case 'e':{
            if(*Len>=2){
                if((Buf[1]=='1')||(Buf[1]=='0')){
                    Main_FilterApSetState(Buf[1]=='1');
                    uint8_t ans[]="ap filter enable=?\n";
                    ans[strlen(ans)-2]=Buf[1];
                    CDC_Transmit_FS(ans, strlen(ans));
                    break;
                }
            }
            uint8_t ans[]="wrong ap filter enable command, use \"e1\" or \"e0\"\n";
            CDC_Transmit_FS(ans, strlen(ans));
            break;
        }
        case 't':{
            if(*Len>=2){
                switch(Buf[1]){
                    case '1':{
                        Main_TeleSetState(1);
                        Main_TelePhaseSetState(0);
                        uint8_t ans[]="tele enable=1\n";
                        CDC_Transmit_FS(ans, strlen(ans));
                        break;
                    }
                    case '0':{
                        Main_TeleSetState(0);
                        Main_TelePhaseSetState(0);
                        uint8_t ans[]="tele enable=0\n";
                        CDC_Transmit_FS(ans, strlen(ans));
                        break;
                    }
                    case 'd':{
                        Main_TeleSetState(0);
                        Main_TelePhaseSetState(1);
                        uint8_t ans[]="tele phase enable=1\n";
                        CDC_Transmit_FS(ans, strlen(ans));
                        break;
                    }
                    case 'b':{
                        Main_TeleSetState(1);
                        Main_TelePhaseSetState(1);
                        uint8_t ans[]="Both tele enable=1\n";
                        CDC_Transmit_FS(ans, strlen(ans));
                        break;
                    }
                    
                }
            }
            uint8_t ans[]="wrong tele enable command, use \"t1\", \"t0\", \"td\"\n";
            CDC_Transmit_FS(ans, strlen(ans));
            break;
        }
        case 'p':{
            uint32_t len=*Len-1;
            uint32_t presc=char2uint(&Buf[1], &len);
            if(Main_PWMPrescSet(presc)){
                uint8_t ans[]="PWM presc set\n";
                CDC_Transmit_FS(ans, strlen(ans));
            }
            else{
                uint8_t ans[]="PWM presc doesn't set\n";
                CDC_Transmit_FS(ans, strlen(ans));
            }
            break;
        }
        
        case 'h':{//help
            uint8_t ans[]="help\n"
            "f12.345 - set sin freq -/- rad/s\n"
            "e1/e0 - on/off filter\n"
            "t1/t0/td/tb - on/off/phase&amp/both tele\n"
            "p123 - set pwm presc -/-\n"
            "z12 - find 0 amp in 12ms(def-1000)\n"
            "dfi - discr filt info\n"
            "dfe1/dfe0 - on/off discr filter\n"
            "dfa3 1.1 -2 0,3 - set df A=[1.1 ...]\n"
            "dfb2 2.1 -1 - set df B=[2.1 -1]\n"
            "m0/1 - set mode of out signal to 0:Single 1: Alternate\n"
            ;
            CDC_Transmit_FS(ans, strlen(ans));
            break;
        }
        case 'd':{
            if(*Len<3){// dfi
                defFlag=true;
                break;
            }
            
            uint32_t iOfSpace=0;
            for(int i=1; i<*Len;++i){
                if(Buf[i]==' '){
                    iOfSpace=i;
                    break;
                }
            }

            if(Buf[1]!='f'||(
                (Buf[2]!='a')
                &&(Buf[2]!='b')
                &&(Buf[2]!='e')
                &&(Buf[2]!='i')
                )){
                defFlag=true;
                break;
            }
            
            if(Buf[2]=='e'){
                defFlag=!Main_FilterDiscrSetState(Buf[3]-'0');
                if(!defFlag){
                    uint8_t ans[]="disc filt e=?\n";
                    ans[strlen(ans)-2]=Buf[3];
                    CDC_Transmit_FS(ans, strlen(ans));
                }
            break;
            }
            
            if(Buf[2]=='i'){
                int ansLen=discFilt_dispLen(discFilt);
                uint8_t ans[ansLen];
                discFilt_disp(discFilt, ans, &ansLen);
                CDC_Transmit_FS(ans, ansLen);
                break;
            }
            //it's a of b key now
            //min string "dfa1 3", where df - discrete filter, a- arr A (numenator), 1- len A polinom, 3- a0 coeff.
            if(*Len<6||iOfSpace==0){
                defFlag=true;
                break;
            }
            //everything is alright
            uint32_t lenLen=iOfSpace-3;//-3 for "dfa"
            uint32_t lenPoly=char2uint(&Buf[3],&lenLen);
            
            discrFilt_setLen(discFilt, lenPoly, Buf[2]);
            uint32_t iPoly=0, numLen=0;
            
            for(int i=iOfSpace+1; (i<*Len)&&(iPoly<lenPoly); ++i){
                if(Buf[i]==' '||i==*Len-1){
                    numLen=i-iOfSpace-1;
                    if(Buf[i]>='0'&&Buf[i]<='9'){//not to lose last digit
                        ++numLen;
                    }
                    discrFilt_setKoef(discFilt, iPoly,
                        char2double(&Buf[iOfSpace+1], &numLen), Buf[2]);
                    ++iPoly;
                    iOfSpace=i;
                }
            }
            int ansLen=(*Len)*2;
            uint8_t ans[ansLen];
            int temp=lenPoly, j=0;
            while(temp>0){
                ans[j]=temp%10 + '0';
                ++j;
                temp/=10;
            }
            ans[j]=':';
            ++j;
            for(int i=0;i<lenPoly&&(j<ansLen);++i){
                temp=discrFilt_getKoef(discFilt,i, Buf[2]);
                do{
                    ans[j]=temp%10 + '0';
                    temp/=10;
                    ++j;
                }while(temp>0);
                ans[j]=' ';
                ++j;
            }
            ans[j-1]='\n';
            //ans[j]='\0';
            ansLen=j;
            
            CDC_Transmit_FS(ans, ansLen);
        }
        case 'z':{
            uint32_t len=*Len-1;
            if(len) Main_FindValZero(1000);
            else{
                Main_FindValZero(char2uint(&Buf[1],&len));
            }
            break;
        }
        case 'm':{
            uint32_t len=*Len-1;
            if(!len){
                defFlag=true;
                break;
            }
            Main_setModeOutput(char2uint(&Buf[1],&len));
            uint8_t ans[]="Ouput mode changed\n";
            CDC_Transmit_FS(ans, strlen(ans));
            break;
        }
        case 's':{
            //'sd'
            if(*Len<2){defFlag=true;break;}
            if(Buf[1]=='d'){//default signal
                Main_setDefaultSignal();
                uint8_t ans[]="default signal was set\n";
                CDC_Transmit_FS(ans, strlen(ans));
            break;
            }
            
            //min string "s1 3", where s - signal, 1- len of signal vals, 3- val.
            if(*Len<4){defFlag=true;break;}
            
            uint32_t iOfSpace=0;
            for(int i=1; i<*Len;++i){
                if(Buf[i]==' '){
                    iOfSpace=i;
                    break;
                }
            }
            
            uint32_t lenLen=iOfSpace-1;// 1 is for "s"
            uint32_t lenSignal=char2uint(&Buf[3],&lenLen);
            static double *signalArr;
            signalArr=(double*)calloc(lenSignal, sizeof(double));
            
            uint32_t iSignal=0, numLen=0;
            
            for(int i=iOfSpace+1; (i<*Len)&&(iSignal<lenSignal); ++i){
                if(Buf[i]==' '||i==*Len-1){
                    numLen=i-iOfSpace-1;
                    if(Buf[i]>='0'&&Buf[i]<='9'){//not to lose last digit
                        ++numLen;
                    }
                    signalArr[iSignal]=char2double(&Buf[iOfSpace+1], &numLen);
                    ++iSignal;
                    iOfSpace=i;
                }
            }
            Main_setCustomSignal(signalArr, lenSignal);
            if(iSignal<lenSignal){ // not working warning
                uint8_t ans[]="signal was set, but last ?? elems empty\n";
                /*
                uint32_t indOfQu=0;
                for(int i=0; i<strlen(ans);++i){
                    if(ans[i]=='?'){
                        indOfQu=i;
                        break;
                    }
                }
                int val=abs(lenSignal-iSignal);
                ans[indOfQu+1]=val%10+'0';
                val/=10;
                ans[indOfQu]=val%10+'0';
                val/=10;
                if(val){
                    ans[indOfQu-1]='>';
                }*/
                CDC_Transmit_FS(ans, strlen(ans));
                break;
            }
            
            uint8_t ans[]="signal was set\n";
            CDC_Transmit_FS(ans, strlen(ans));
            break;
        }
        case 'a':{//min pulse
            uint32_t len=*Len-1;
            if(!len){
                defFlag=true;
                break;
            }
            double pulse=char2double(&Buf[1], &len);
            if(pulse>100){
                pulse=100;
            }
            if(pulse<0){
                pulse=0;
            }
            
            Main_setPulseMin(pulse);
            
            uint8_t ans[]="min pulse changed\n";
            CDC_Transmit_FS(ans, strlen(ans));
        }
        default:{
            defFlag=true;
            break;
        }
    }
    
  if(defFlag){
    uint8_t ans[]="wrong key\n";
    CDC_Transmit_FS(ans, strlen(ans));
  }
  
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
