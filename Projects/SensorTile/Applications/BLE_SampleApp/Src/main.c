/**
******************************************************************************
* @file    main.c
* @author  SRA - Central Labs
* @version V2.0.0
* @date    5-Feb-2019
* @brief   Main program body
******************************************************************************
* @attention
*
* <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
*
* Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
* You may not use this file except in compliance with the License.
* You may obtain a copy of the License at:
*
*        http://www.st.com/software_license_agreement_liberty_v2
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

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <math.h>
#include <limits.h>
#include "TargetFeatures.h"
#include "main.h"
#include "sensor_service.h"
#include "bluenrg_utils.h"
#include "bluenrg_gatt_aci.h"
#include "mci_hw_gpio.h"
#include "mci_hw_timer.h"
// used by bleUART
#include "bluenrg_gatt_server.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Imported Variables -------------------------------------------------------------*/
extern uint8_t set_connectable;
extern int connected;

extern TIM_HandleTypeDef TimHandle;
extern void CDC_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


/* Exported Variables -------------------------------------------------------------*/

uint32_t ConnectionBleStatus  =0;

uint8_t BufferToWrite[256];
int32_t BytesToWrite;

TIM_HandleTypeDef TimCCHandle;

uint8_t bdaddr[6];

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t SendEnv = 0;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);

static void Init_BlueNRG_Custom_Services(void);
static void Init_BlueNRG_Stack(void);
static void InitTimers(void);
static void SendEnvironmentalData(void);


t_ble_uart_struct bud;
uint16_t ble_UART_serv_Handle, TXCharHandle, RXCharHandle;
//uint8_t CDC_Fill_Buffer(uint8_t* Buf, uint32_t TotalLen){}

/**
* @brief  Main program
* @param  None
* @retval None
**/
void inline ble_uart_tx (uint16_t ble_UART_serv_Handle,uint16_t TXCharHandle,uint16_t len, uint8_t *cmd);
/*100ms ISR */
void TIM3_IRQHandler (void) 
	{
	static uint8_t led_toggle_delay = 500 /* period in ms */ / 100 /* isr occurence period*/ ;
		
  if (TIM3->SR & (TIM_SR_UIF)) /* UIF set? */
		{
    TIM3->SR &= ~(TIM_SR_UIF); /* clear UIF flag */
		

		if (led_toggle_delay)
		{
			led_toggle_delay--;
		}
		else
		{
		led_toggle_delay = 500 / 100;
		uint8_t somebuff[] = {0x55,0x11,0x55,0x55,0x11,0x55,0x55,0x11};
		somebuff[7]+= somebuff[6]--;
		somebuff[5]+= somebuff[4]--;
		somebuff[3]+= somebuff[2]--;
		
		aci_gatt_update_char_value(ble_UART_serv_Handle,TXCharHandle, 1, 8, &somebuff[0]);
		#define LEDS_USED_TO_DEBUG
		#ifndef LEDS_USED_TO_DEBUG
		GPIO_TOGGLE_SWD_LED;
		if (connected)
			GPIO_TOGGLE_ORANGE_LED;
		#endif
		}
				
		if ( ( bud.state & TO_SEND ) && ( bud.state & BLE_UART_ACTIVE ) )
		{
			ble_uart_tx(ble_UART_serv_Handle, TXCharHandle, bud.len, bud.rx_data);
			GPIO_TOGGLE_ORANGE_LED;
		}
		//aci_gatt_update_char_value_ext_IDB05A1();
		// just send it back what we got.

		}
}


#define DISCOVERY_MODE 		0x01
#define	ADV_MODE			 		0x02
#define BLE_UART_MODE 		0x03

void init_blue_nrg_ms_stack(uint8_t mode_select)
{	
	init_tim3();
	init_gpio_leds();
	uint8_t hwVersion = 0;
	uint16_t fwVersion = 0;	
	uint16_t service_handle, dev_name_char_handle, appearance_char_handle = 0;
	uint8_t bdaddr[] = {0xaa, 0x00, 0x00, 0xE1, 0x80, 0x02};
	uint8_t status = 0;

	hci_init(HCI_Event_CB, NULL);
	getBlueNRGVersion(&hwVersion, &fwVersion);
	// reset BlueNRG
	status = aci_gatt_init();
	//status = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET, CONFIG_DATA_PUBADDR_LEN, bdaddr);	

  status = aci_gap_init_IDB05A1(GAP_CENTRAL_ROLE_IDB05A1 | GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
	
	// as gap_init_seems to overwrite the ADDR we need to set it after calling gap_init
	status = hci_le_set_random_address(bdaddr);


	if (mode_select == DISCOVERY_MODE)
	{
		status = aci_gap_start_general_discovery_proc(0x004 /*2.5ms */, 0x004 /* 2.5ms */, 0x01 /* private address */, 0x00 /* do not filter duplicates */ );
		//aci_gap_terminate_gap_procedure(
	
		static int debugval1 = 0;
		switch (status)
		{
			case 0x00: /* SUCCESS */
				debugval1++; /*just for the debugger to jump in */
				break;
			case 0x12: /* INVALID PARAM */
				debugval1--; /*just for the debugger to jump in */
				break;
			case 0x0C: /* ERR CMD DISALLOWED */
				debugval1--; /*just for the debugger to jump in */
				break;
		}
	}
	if (mode_select == ADV_MODE)
	{
		const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'C','a','t','c','h','?','!' };
		/* As scan response data, a proprietary 128bits Service UUID is used.
		This 128bits data cannot be inserted within the advertising packet
		(ADV_IND) due its length constraints (31 bytes). */
		/* 
			AD Type description:
			0x11: length
			0x06: 128 bits Service UUID type
			0x8a,0x97,0xf7,0xc0,0x85,0x06,0x11,0xe3,0xba,0xa7,0x08,0x00,0x20,0x0c,0x9a,0x66: 128 bits Service UUID 
		*/
		uint8_t ServiceUUID_Scan[18]= {0x11,0x06,0x8a,0x97,0xf7,0xc0,0x85,0x06,0x11,0xe3,0xba,0xa7,0x08,0x00,0x20,0x0c,0x9a,0x66};
		/* Enable scan response to be sent when GAP peripheral receives
		scan requests from GAP Central performing general
		discovery procedure(active scan) */
			
		hci_le_set_scan_resp_data(18 , ServiceUUID_Scan);
		
		/* 
		Put the GAP peripheral in general discoverable mode:
		Advertising_Event_Type: ADV_IND (undirected scannable and connectable); / ADV_SCAN_IND (scannable but not connectable)
		Adv_Interval_Min: 0;
		Adv_Interval_Max: 0;
		Address_Type: PUBLIC_ADDR (public address: 0x00);
		Adv_Filter_Policy: NO_WHITE_LIST_USE (no whit list is used);
		Local_Name_Length: 8
		Local_Name: BlueNRG;
		Service_Uuid_Length: 0 (no service to be advertised);
		Service_Uuid_List: NULL;
		Slave_Conn_Interval_Min: 0 (Slave connection internal minimum value);
		Slave_Conn_Interval_Max: 0 (Slave connection internal maximum value).
		*/
            
        HAL_Delay(2500);
		//status = aci_gap_set_discoverable(ADV_SCAN_IND, 0x0020, 0x0020, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);
   		status = aci_gap_set_discoverable(ADV_SCAN_IND, 0x0001, 0x0001, STATIC_RANDOM_ADDR, NO_WHITE_LIST_USE, sizeof(local_name), local_name, 0, NULL, 0, 0);

        GPIO_SET_PC0;
        GPIO_SET_PC1;
	}
	
	
	if (mode_select == BLE_UART_MODE)
	{
		const char local_name[] = {AD_TYPE_COMPLETE_LOCAL_NAME,'C','a','t','c','h','?','!' };
		
		const uint8_t service_uuid[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe0,0xf2,0x73,0xd9};
		const uint8_t charUuidTX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe1,0xf2,0x73,0xd9};
		const uint8_t charUuidRX[16] = {0x66,0x9a,0x0c,0x20,0x00,0x08,0x96,0x9e,0xe2,0x11,0x9e,0xb1,0xe2,0xf2,0x73,0xd9};
		
		aci_gatt_add_serv(UUID_TYPE_128, service_uuid, PRIMARY_SERVICE, 7, &ble_UART_serv_Handle);
		
		// Add TX Characteristic with Notification property (The Central device gets notified)
		/*ServHandle, Char_UUID_Type, Char_UUID_16, Char_Value_Length, Char_Properties, Security_Permissions, GATT_Evt_Mask, Enc_Key_Size, Is_Variable, &CharHandle*/
		aci_gatt_add_char(ble_UART_serv_Handle, UUID_TYPE_128, charUuidTX, 20, CHAR_PROP_NOTIFY, ATTR_PERMISSION_NONE, 0, 16, 1, &TXCharHandle);
		
		// Set RX Cbaracteristic
		/* The Central can write it without resp, no security permissions(ATTR_PERMISSION_NONE), a GATT event mask (0x01), 
		16 as keyencryption size, and variable-length characteristic (1). 
		The characteristic handle is returned. */
		/*ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, uuid, 2+1,
                           CHAR_PROP_NOTIFY | CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &LedCharHandle)*/
		aci_gatt_add_char(ble_UART_serv_Handle, UUID_TYPE_128, charUuidRX, 20, CHAR_PROP_WRITE | CHAR_PROP_WRITE_WITHOUT_RESP, ATTR_PERMISSION_NONE, EVT_BLUE_GATT_ATTRIBUTE_MODIFIED, 16, 1, &RXCharHandle);
		//aci_gatt_update_char_value_ext_IDB05A1(
		//aci_gatt_add_char_desc(ble_UART_serv_Handle,
		
		// go discoverable (peripheral role, wait for a central device to connect)
		aci_gap_set_discoverable(ADV_IND, 0, 0, PUBLIC_ADDR, NO_WHITE_LIST_USE,8, local_name, 0, NULL, 0,0);

	}
}




void inline ble_uart_tx (uint16_t ble_UART_serv_Handle,uint16_t TXCharHandle,uint16_t len, uint8_t *cmd)
{
	/*
	tBleStatus aci_gatt_update_char_value(uint16_t servHandle, 
				      uint16_t charHandle,
				      uint8_t charValOffset,
				      uint8_t charValueLen,   
              const void *charValue)
	*/
	aci_gatt_update_char_value(ble_UART_serv_Handle,TXCharHandle, 1, len, cmd);
	/* 
	Why +1?!
	The offset from which the attribute value has to be updated. If
	this is set to 0, and the attribute value is of variable length, then
	the length of the attribute will be set to the Char_Value_Length. If
	the Val_Offset is set to a value greater than 0, then the length of
	the attribute will be set to the maximum length as specified for
	the attribute while adding the characteristic.
	*/
}

int main(void)
{
  uint32_t StartTime;
  
  /* STM32L4xx HAL library initialization:
  - Configure the Flash prefetch, instruction and Data caches
  - Configure the Systick to generate an interrupt each 1 msec
  - Set NVIC Group Priority to 4
  - Global MSP (MCU Support Package) initialization
  */
  HAL_Init();
  
  /* Configure the System clock */
  SystemClock_Config();
  
  InitTargetPlatform(TARGET_SENSORTILE);
  

  /* Initialize the BlueNRG */
  //Init_BlueNRG_Stack();
  
  /* Initialize the BlueNRG Custom services */
  //Init_BlueNRG_Custom_Services();  
  init_gpio_pc0_as_output();
  init_gpio_pc1_as_output();
  GPIO_CLEAR_PCO;
  GPIO_CLEAR_PC1;
  
  init_blue_nrg_ms_stack(BLE_UART_MODE);
  
  /* initialize timers */
  InitTimers();
  
  StartTime = HAL_GetTick();
	
	while(1)
	{
			if(HAL_GetTick()-StartTime > 5)
			{
				StartTime = HAL_GetTick();
				
				if(HCI_ProcessEvent)
				{					

				HCI_ProcessEvent=0;
				hci_user_evt_proc();
				}
				
				if (connected)
				{
					bud.state |= BLE_UART_ACTIVE;
				}
				else
				{
					bud.state &= !BLE_UART_ACTIVE;
				}
			}
	}
	
  /* Infinite loop */
  while (1)
  {
    /* Led Blinking when there is not a client connected */
    if(!connected) 
    {
      if(!TargetBoardFeatures.LedStatus) 
      {
        if(HAL_GetTick()-StartTime > 1000)
        {
          LedOnTargetPlatform();
          TargetBoardFeatures.LedStatus =1;
          StartTime = HAL_GetTick();
        }
      } 
      else 
      {
        if(HAL_GetTick()-StartTime > 50)
        {
          LedOffTargetPlatform();
          TargetBoardFeatures.LedStatus =0;
          StartTime = HAL_GetTick();
        }
      }
    }
    
    /* handle BLE event */
    if(HCI_ProcessEvent) 
    {
      HCI_ProcessEvent=0;
      hci_user_evt_proc();
    }
    
    /* Update the BLE advertise data and make the Board connectable */
    if(set_connectable)
    {
      setConnectable();
      set_connectable = FALSE;
    }
    
    /* Environmental Data */
    if(SendEnv) 
    {
      SendEnv=0;
      SendEnvironmentalData();
    }
    
    /* Wait for Interrupt */
    __WFI();
  }
}

/**
* @brief  Output Compare callback in non blocking mode 
* @param  htim : TIM OC handle
* @retval None
*/
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t uhCapture=0;
  
  /* TIM1_CH1 toggling with frequency = 2Hz */
  if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
  {
    uhCapture = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    /* Set the Capture Compare Register value */
    __HAL_TIM_SET_COMPARE(&TimCCHandle, TIM_CHANNEL_1, (uhCapture + uhCCR1_Val));
    SendEnv=1;
  }
}

/**
* @brief  Period elapsed callback in non blocking mode for Environmental timer
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim == (&TimHandle)) 
  {
    CDC_TIM_PeriodElapsedCallback(htim);
  }
}

/**
* @brief  Send Environmetal Data (Temperature/Pressure/Humidity) to BLE
* @param  None
* @retval None
*/
static void SendEnvironmentalData(void)
{  
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
  {
    BytesToWrite = sprintf((char *)BufferToWrite,"Sending: ");
    Term_Update(BufferToWrite,BytesToWrite);
  } 
  else 
  {
    STLBLE_PRINTF("Sending: ");
  }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
  
  /* Pressure,Humidity, and Temperatures*/
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_ENV)) 
  {
    float SensorValue;
    int32_t PressToSend=0;
    uint16_t HumToSend=0;
    int16_t Temp2ToSend=0,Temp1ToSend=0;
    int32_t decPart, intPart;
    
    if(TargetBoardFeatures.HandlePressSensor) 
    {
      BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_PRESSURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_2D(SensorValue, intPart, decPart);
      PressToSend=intPart*100+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) 
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Press=%ld ",PressToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } 
      else 
      {
        STLBLE_PRINTF("Press=%ld ",PressToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */      
    }
    
    if(TargetBoardFeatures.HandleHumSensor)
    {
      
      BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_HUMIDITY, (float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      HumToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) 
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Hum=%d ",HumToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } 
      else 
      {
        STLBLE_PRINTF("Hum=%d ",HumToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */      
    }
    
    if(TargetBoardFeatures.NumTempSensors==2) 
    {
      BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp1ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM)) 
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp=%d ",Temp1ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } 
      else 
      {
        STLBLE_PRINTF("Temp=%d ",Temp1ToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
      
      BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE,(float *)&SensorValue);
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp2ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp2=%d ",Temp2ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      } 
      else 
      {
        STLBLE_PRINTF("Temp2=%d ",Temp2ToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */      
    } 
    else if(TargetBoardFeatures.NumTempSensors==1)
    {
      if (BSP_ENV_SENSOR_GetValue(HTS221_0, ENV_TEMPERATURE,(float *)&SensorValue)!=BSP_ERROR_NONE)
      {
        BSP_ENV_SENSOR_GetValue(LPS22HB_0, ENV_TEMPERATURE,(float *)&SensorValue);
      }
      MCR_BLUEMS_F2I_1D(SensorValue, intPart, decPart);
      Temp1ToSend = intPart*10+decPart;
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
      if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
      {
        BytesToWrite = sprintf((char *)BufferToWrite,"Temp1=%d ",Temp1ToSend);
        Term_Update(BufferToWrite,BytesToWrite);
      }
      else
      {
        STLBLE_PRINTF("Temp1=%d ",Temp1ToSend);
      }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
      
    }
    Environmental_Update(PressToSend,HumToSend,Temp2ToSend,Temp1ToSend);
  }
  
#ifdef ENABLE_USB_DEBUG_NOTIFY_TRAMISSION
  if(W2ST_CHECK_CONNECTION(W2ST_CONNECT_STD_TERM))
  {
    BytesToWrite = sprintf((char *)BufferToWrite,"\r\n");
    Term_Update(BufferToWrite,BytesToWrite);
  }
  else 
  {
    STLBLE_PRINTF("\r\n");
  }
#endif /* ENABLE_USB_DEBUG_NOTIFY_TRAMISSION */
}

/**
* @brief  Function for initializing timers for sending the information to BLE:
*  - 1 for sending MotionFX/AR/CP and Acc/Gyro/Mag
*  - 1 for sending the Environmental info
* @param  None
* @retval None
*/
static void InitTimers(void)
{
  uint32_t uwPrescalerValue;
  
  /* Timer Output Compare Configuration Structure declaration */
  TIM_OC_InitTypeDef sConfig;
  
  /* Compute the prescaler value to have TIM3 counter clock equal to 10 KHz */
  uwPrescalerValue = (uint32_t) ((SystemCoreClock / 10000) - 1); 
  
  /* Set TIM1 instance (Motion)*/
  /* Set TIM1 instance */
  TimCCHandle.Instance = TIM1;
  TimCCHandle.Init.Period        = 65535;
  TimCCHandle.Init.Prescaler     = uwPrescalerValue;
  TimCCHandle.Init.ClockDivision = 0;
  TimCCHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;
  if(HAL_TIM_OC_Init(&TimCCHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
  
  /* Configure the Output Compare channels */
  /* Common configuration for all channels */
  sConfig.OCMode     = TIM_OCMODE_TOGGLE;
  sConfig.OCPolarity = TIM_OCPOLARITY_LOW;
  
  /* Output Compare Toggle Mode configuration: Channel1 */
  sConfig.Pulse = uhCCR1_Val;
  if(HAL_TIM_OC_ConfigChannel(&TimCCHandle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
  {
    /* Configuration Error */
    Error_Handler();
  }  
  
}

/** @brief Initialize the BlueNRG Stack
* @param None
* @retval None
*/
static void Init_BlueNRG_Stack(void)
{
  const char BoardName[8] = {NAME_STLBLE,0};
  uint16_t service_handle, dev_name_char_handle, appearance_char_handle;
  int ret;
  uint8_t  hwVersion;
  uint16_t fwVersion;
  
#ifdef STATIC_BLE_MAC
  {
    uint8_t tmp_bdaddr[6]= {STATIC_BLE_MAC};
    int32_t i;
    for(i=0;i<6;i++)
      bdaddr[i] = tmp_bdaddr[i];
  }
#endif /* STATIC_BLE_MAC */

  /* Initialize the BlueNRG HCI */
  hci_init(HCI_Event_CB, NULL);
  
  /* get the BlueNRG HW and FW versions */
  getBlueNRGVersion(&hwVersion, &fwVersion);
  
#ifndef STATIC_BLE_MAC
  /* Create a Unique BLE MAC */
  {
    bdaddr[0] = (STM32_UUID[1]>>24)&0xFF;
    bdaddr[1] = (STM32_UUID[0]    )&0xFF;
    bdaddr[2] = (STM32_UUID[2] >>8)&0xFF;
    bdaddr[3] = (STM32_UUID[0]>>16)&0xFF;
    bdaddr[4] = (((STLBLE_VERSION_MAJOR-48)*10) + (STLBLE_VERSION_MINOR-48)+100)&0xFF;
    bdaddr[5] = 0xC0; /* for a Legal BLE Random MAC */
  }
#else /* STATIC_BLE_MAC */
  
  ret = aci_hal_write_config_data(CONFIG_DATA_PUBADDR_OFFSET,
                                  CONFIG_DATA_PUBADDR_LEN,
                                  bdaddr);
  if(ret)
  {
    STLBLE_PRINTF("\r\nSetting Pubblic BD_ADDR failed\r\n");
    goto fail;
  }
#endif /* STATIC_BLE_MAC */
  
  ret = aci_gatt_init();    
  if(ret)
  {
    STLBLE_PRINTF("\r\nGATT_Init failed\r\n");
    goto fail;
  }
  

    ret = aci_gap_init_IDB05A1(GAP_PERIPHERAL_ROLE_IDB05A1, 0, 0x07, &service_handle, &dev_name_char_handle, &appearance_char_handle);
  
  if(ret != BLE_STATUS_SUCCESS)
  {
    STLBLE_PRINTF("\r\nGAP_Init failed\r\n");
    goto fail;
  }
  
#ifndef  STATIC_BLE_MAC
  ret = hci_le_set_random_address(bdaddr);
  
  if(ret)
  {
    STLBLE_PRINTF("\r\nSetting the Static Random BD_ADDR failed\r\n");
    goto fail;
  }
#endif /* STATIC_BLE_MAC */
  
  ret = aci_gatt_update_char_value(service_handle, dev_name_char_handle, 0,
                                   7/*strlen(BoardName)*/, (uint8_t *)BoardName);
  
  if(ret)
  {
    STLBLE_PRINTF("\r\naci_gatt_update_char_value failed\r\n");
    while(1);
  }
  
  ret = aci_gap_set_auth_requirement(MITM_PROTECTION_REQUIRED,
                                     OOB_AUTH_DATA_ABSENT,
                                     NULL, 7, 16,
                                     USE_FIXED_PIN_FOR_PAIRING, 123456,
                                     BONDING);
  if (ret != BLE_STATUS_SUCCESS) 
  {
    STLBLE_PRINTF("\r\nGAP setting Authentication failed\r\n");
    goto fail;
  }
  
  STLBLE_PRINTF("SERVER: BLE Stack Initialized \r\n"
                "\t\tBoard type=%s HWver=%d, FWver=%d.%d.%c\r\n"
                  "\t\tBoardName= %s\r\n"
                    "\t\tBoardMAC = %x:%x:%x:%x:%x:%x\r\n\n",
                    "SensorTile",
                    hwVersion,
                    fwVersion>>8,
                    (fwVersion>>4)&0xF,
                    (hwVersion > 0x30) ? ('a'+(fwVersion&0xF)-1) : 'a',
                    BoardName,
                    bdaddr[5],bdaddr[4],bdaddr[3],bdaddr[2],bdaddr[1],bdaddr[0]);
  
  /* Set output power level */
  aci_hal_set_tx_power_level(1,4);
  
  return;
  
fail:
  return;
}

/** @brief Initialize all the Custom BlueNRG services
* @param None
* @retval None
*/
static void Init_BlueNRG_Custom_Services(void)
{
  int ret;
  
  ret = Add_HWServW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) 
  {
    STLBLE_PRINTF("HW      Service W2ST added successfully\r\n");
  } 
  else 
  {
    STLBLE_PRINTF("\r\nError while adding HW Service W2ST\r\n");
  }
  
  ret = Add_ConfigW2ST_Service();
  if(ret == BLE_STATUS_SUCCESS) 
  {
    STLBLE_PRINTF("Config  Service W2ST added successfully\r\n");
  } 
  else 
  {
    STLBLE_PRINTF("\r\nError while adding Config Service W2ST\r\n");
  }
}

/**
* @brief  System Clock Configuration
* @param  None
* @retval None
*/
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  
  /* Enable the LSE Oscilator */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }
  
  /* Enable the CSS interrupt in case LSE signal is corrupted or not present */
  HAL_RCCEx_DisableLSECSS();
  
  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState            = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange       = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM            = 6;
  RCC_OscInitStruct.PLL.PLLN            = 40;
  RCC_OscInitStruct.PLL.PLLP            = 7;
  RCC_OscInitStruct.PLL.PLLQ            = 4;
  RCC_OscInitStruct.PLL.PLLR            = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }
  
  /* Enable MSI Auto-calibration through LSE */
  HAL_RCCEx_EnableMSIPLLMode();
  
  /* Select MSI output as USB clock source */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_MSI;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    while(1);
  }
}


/**
* @brief This function provides accurate delay (in milliseconds) based 
*        on variable incremented.
* @note This is a user implementation using WFI state
* @param Delay: specifies the delay time length, in milliseconds.
* @retval None
*/
void HAL_Delay(__IO uint32_t Delay)
{
  uint32_t tickstart = 0;
  tickstart = HAL_GetTick();
  while((HAL_GetTick() - tickstart) < Delay)
  {
    __WFI();
  }
}


/**
* @brief  This function is executed in case of error occurrence.
* @param  None
* @retval None
*/
void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
    
  }
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
  /* User can add his own implementation to report the file name and line number,
  ex: STLBLE_PRINTF("Wrong parameters value: file %s on line %d\r\n", file, line) */
  
  /* Infinite loop */
  while (1){
  }
}
#endif


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
