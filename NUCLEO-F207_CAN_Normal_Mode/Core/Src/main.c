/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define STM32_FAMILY_ID 0xF2                     /* For STM32F2 devices */
#define BOARD_ID        0x0                      /* BOARD_ID: from 0 to 7: Device number if two
                                                    boards using STM32F2, need to select another value (1 to 7) */
#define TX_ID  ((BOARD_ID<<8) | STM32_FAMILY_ID) /* TX ID to send is a format of 0xXF2, where X is the board ID */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN PV */
uint32_t              TxMailbox;       /* The number of the mail box that transmitted the Tx message */
CAN_TxHeaderTypeDef   TxHeader;        /* Header containing the information of the transmitted frame */
uint8_t               TxData[8] = {0}; /* Buffer of the data to send */

CAN_RxHeaderTypeDef   RxHeaderFIFO0;   /* Header containing the information of the received frame */
CAN_RxHeaderTypeDef   RxHeaderFIFO1;   /* Header containing the information of the received frame */
uint8_t               RxDataFIFO0[8];  /* Buffer of the received data */
uint8_t               RxDataFIFO1[8];  /* Buffer of the received data */
__IO uint32_t         rx_f0_std = 0;   /* Contains the number of standard frames received from a STM32F0 MCU */
__IO uint32_t         rx_f0_ext = 0;   /* Contains the number of extended frames received from a STM32F0 MCU */
__IO uint32_t         rx_f1_std = 0;   /* Contains the number of standard frames received from a STM32F1 MCU */
__IO uint32_t         rx_f1_ext = 0;   /* Contains the number of extended frames received from a STM32F1 MCU */
__IO uint32_t         rx_f3_std = 0;   /* Contains the number of standard frames received from a STM32F3 MCU */
__IO uint32_t         rx_f3_ext = 0;   /* Contains the number of extended frames received from a STM32F3 MCU */
__IO uint32_t         rx_f4_std = 0;   /* Contains the number of standard frames received from a STM32F4 MCU */
__IO uint32_t         rx_f4_ext = 0;   /* Contains the number of extended frames received from a STM32F4 MCU */
__IO uint32_t         rx_f7_std = 0;   /* Contains the number of standard frames received from a STM32F7 MCU */
__IO uint32_t         rx_f7_ext = 0;   /* Contains the number of extended frames received from a STM32F7 MCU */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_CAN1_Init();
  /* USER CODE BEGIN 2 */
  TxHeader.StdId = TX_ID;              /* The ID value in Standard ID format (11bit) */
  TxHeader.ExtId = TX_ID | 0x10000000; /* The ID value in Extended ID format (29bit) */
  TxHeader.RTR = CAN_RTR_DATA;         /* The frames that will be sent are Data */
  TxHeader.DLC = 8;                    /* The frames will contain 8 data bytes */
  TxHeader.TransmitGlobalTime = DISABLE;

  /* Only the first byte (data0) and the last byte (data7) will be changed */
  TxData[0] = 0;                       /* The first value to send on byte 0 is 0 */
  TxData[7] = 0xFF;                    /* The last value to send on byte 0 is 0xFF */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  TxData[0] ++; /* Increment the first byte */
      TxData[7] --; /* Increment the last byte */
      /* It's mandatory to look for a free Tx mail box */
      while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0) {} /* Wait till a Tx mailbox is free. Using while loop instead of HAL_Delay() */

      if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK) /* Send the CAN frame */
      {
          /* Transmission request Error */
          Error_Handler();
      }

      /* Toggle sending Standard and Extended ID for the next operation. The first ID sent is a Standard ID frame */
      TxHeader.IDE ^= CAN_ID_EXT;
      HAL_Delay(1);
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 240;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 2;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  /* The filter configuration */
  sFilterConfig.SlaveStartFilterBank = 14;           /* Slave start bank Set only once. */

  /* The CAN filter configuration */
  sFilterConfig.FilterBank = 0;                      /* Select the filter number 0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* Using ID mask mode .. */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* .. in 32-bit scale */
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;                /* The filter is set to receive only the Standard ID frames */
  sFilterConfig.FilterMaskIdHigh = 0x0000;           /* Accept all the IDs .. except the Extended frames */
  sFilterConfig.FilterMaskIdLow = 0x0004;            /* The filter is set to check only on the ID format */
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0; /* All the messages accepted by this filter will be received on FIFO1 */
  sFilterConfig.FilterActivation = ENABLE;           /* Enable the filter number 0 */
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
      /* Filter configuration Error */
      Error_Handler();
  }

  sFilterConfig.FilterBank = 1;                      /* Select the filter number 1 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* Using ID mask mode .. */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* .. in 32-bit scale */
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0004;                /* The filter is set to receive only the Extended ID frames */
  sFilterConfig.FilterMaskIdHigh = 0x0000;           /* Accept all the IDs .. except the Standard frames */
  sFilterConfig.FilterMaskIdLow = 0x0004;            /* The filter is set to check only on the ID format */
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO1; /* All the messages accepted by this filter will be received on FIFO1 */
  sFilterConfig.FilterActivation = ENABLE;           /* Enable the filter number 1 */
  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK)
  {
      /* Filter configuration Error */
      Error_Handler();
  }

  /* Start the CAN peripheral */
  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
      /* Start Error */
      Error_Handler();
  }

  /* Activate CAN RX notifications on FIFO0 and on FIFO1 */
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
  {
      /* Notification Error */
      Error_Handler();
  }
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LD1_Pin LD3_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* CAN callback of FIF00. It will be called each time FIF00 receives a frame.
  As a filter has been set to receive only Standard Id frames on FIFO0, This callback
  will be called only when the Standard ID frame is received */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    /* Get RX message from FIFO0 and fill the data on the related FIFO0 user declared header
       (RxHeaderFIFO0) and table (RxDataFIFO0) */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO0, &RxHeaderFIFO0, RxDataFIFO0) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }
    else
    {
        /* Toggle the LED when a message is received. Note that the toggling is not seen as the
           transmit/receive of the frames are performed in a high frequency.
           If you need to see the LED toggling, you need to increase the delay that was called just after 
		   HAL_CAN_AddTxMessage() from the sender side for example HAL_Delay(500) */
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }

    if ((RxHeaderFIFO0.StdId & 0xFF) == 0xF0)
    {
        rx_f0_std++; /* Increment this if a standard CAN frame is received from the NUCLEO-F042 board */
    }
    else if ((RxHeaderFIFO0.StdId & 0xFF) == 0xF1)
    {
        rx_f1_std++;  /* Increment this if a standard CAN frame is received from the NUCLEO-F103 board */
    }
    else if ((RxHeaderFIFO0.StdId & 0xFF) == 0xF3)
    {
    	rx_f3_std++; /* Increment this if a standard CAN frame is received from the NUCLEO-F302 board */
    }
    else if ((RxHeaderFIFO0.StdId & 0xFF) == 0xF4)
    {
        rx_f4_std++; /* Increment this if a standard CAN frame is received from the NUCLEO-F439 board */
    }
    else if ((RxHeaderFIFO0.StdId & 0xFF) == 0xF7)
    {
    	rx_f7_std++; /* Increment this if a standard CAN frame is received from the NUCLEO-F767 board */
    }
}

/* CAN callback of FIF01. It will be called each time FIF01 receives a frame.
  As a filter has been set to receive only Extended ID frames on FIFO1, This callback
  will be called only when an Extended ID frame is received */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
    /* Get RX message from FIFO1 and fill the data on the related FIFO1 user declared header
       (RxHeaderFIFO1) and table (RxDataFIFO1) */
    if (HAL_CAN_GetRxMessage(CanHandle, CAN_RX_FIFO1, &RxHeaderFIFO1, RxDataFIFO1) != HAL_OK)
    {
        /* Reception Error */
        Error_Handler();
    }
    else
    {
        /* Toggle the LED when a message is received. Note that the toggling is not seen as the
           transmit/receive of the frames are performed in a high frequency.
           If you need to see the LED toggling, you need to increase the delay that was called just after 
		   HAL_CAN_AddTxMessage() from the sender side for example HAL_Delay(500) */
        HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    }

    if ((RxHeaderFIFO1.ExtId & 0xFF) == 0xF0)
    {
        rx_f0_ext++; /* Increment this if an Extended CAN frame is received from the NUCLEO-F042 board */
    }
    else if ((RxHeaderFIFO1.ExtId & 0xFF) == 0xF1)
    {
        rx_f1_ext++; /* Increment this if an Extended CAN frame is received from the NUCLEO-F103 board */
    }
    else if ((RxHeaderFIFO1.ExtId & 0xFF) == 0xF3)
    {
    	rx_f3_ext++; /* Increment this if an Extended CAN frame is received from the NUCLEO-F302 board */
    }
    else if ((RxHeaderFIFO1.ExtId & 0xFF) == 0xF4)
    {
    	rx_f4_ext++; /* Increment this if an Extended CAN frame is received from the NUCLEO-F439 board */
    }
    else if ((RxHeaderFIFO1.ExtId & 0xFF) == 0xF7)
    {
    	rx_f7_ext++; /* Increment this if an Extended CAN frame is received from the NUCLEO-F767 board */
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
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
      /* Use this delay instead of HAL_Delay() since the interrupts has been disabled */
      for(__IO uint32_t i = 0; i < 0xFFFFF; i++);
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
