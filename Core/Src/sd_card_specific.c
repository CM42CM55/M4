/*
 * sd_card_specific.c
 *
 *  Created on: Jul 10, 2024
 *      Author: Ankesh
 */


#include "main.h"
#include "fatfs.h"
#include "stm32f4xx_hal.h"
#include "bsp_driver_sd.h"

//#define MSD_OK 				((unit8_t) 0x00)
//#define MSD_ERROR			((uint8_t) 0x01)

//#define SD_TRANSFER_OK		((unit8_t) 0x00)
//#define SD_TRANSFER_BUSY	((uint8_t) 0x01)

//#define	SD_DATATIMEOUT		((uint32_t) 100000000)
//#define SD_PRESENT			((unit8_t) 0x00)
//#define SD_NOT_PRESENT		((uint8_t) 0x01)


void bsp_sdio_init(void);
static void MX_SDIO_SD_Init(void);
static void MX_DMA_Init(void);
static void FAT_FUNCTION(void);
void Error_Handler(void);

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;


FATFS SDFAT_FS;
FIL MY_FILE;

char SD_PATH[4];
static uint8_t buffer[_MAX_SS];




__weak void bsp_sdio_init(void)	{

	GPIO_InitTypeDef GPIO_Init_Structure;

	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();

	/* Common GPIO configuration */
	GPIO_Init_Structure.Mode		= GPIO_MODE_AF_PP;
	GPIO_Init_Structure.Pull		= GPIO_PULLUP;
	GPIO_Init_Structure.Speed		= GPIO_SPEED_HIGH;
	GPIO_Init_Structure.Alternate	= GPIO_AF12_SDIO;

	/* GPIOC configuration */
	GPIO_Init_Structure.Pin			= GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;

	/* GPIOC INIT */
	HAL_GPIO_Init(GPIOC, &GPIO_Init_Structure);

	/* GPIOD configuration */
	GPIO_Init_Structure.Pin			= GPIO_PIN_2;

	/* GPIOD INIT */
	HAL_GPIO_Init(GPIOD, &GPIO_Init_Structure);
}


/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  * This is for external SDIO interface connected to APB bus @ SDIO_BASE(Not for AMS Board)
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_4B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}




/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}



static void FAT_FUNCTION(void)	{
	FRESULT res;
	uint32_t byteswritten, bytesread;
	uint8_t wtext[] = "This is FAT_FS example";
	uint8_t rtext[100];

	if(FATFS_LinkDriver(&SD_Driver, SDPath) == 0)	{
		if(f_mount(&SDFatFS, (TCHAR const *) SDPath, 0) != FR_OK)	{
			Error_Handler();
		}	else	{
			if(f_mkfs((TCHAR const *)SDPath, FM_ANY, 0, buffer, sizeof(buffer)) != FR_OK)	{
				Error_Handler();
			}	else	{
				res = f_write(&MY_FILE, wtext, sizeof(wtext), (void *)&byteswritten);
				if((byteswritten == 0) || (res != FR_OK))	{
					Error_Handler();
				}	else	{
					f_close(&MY_FILE);
					if(f_open(&MY_FILE, "STM32.TXT", FA_READ) != FR_OK)	{
						Error_Handler();
					}	else	{
						res = f_read(&MY_FILE, rtext, sizeof(rtext), (UINT *)&bytesread);
						if(bytesread == 0 || res != FR_OK)	{
							Error_Handler();
						}	else	{
							f_close(&MY_FILE);
							if(byteswritten != bytesread)	{
								Error_Handler();
							}	else	{
								//LED Glow to be written
							}
						}
					}
				}
			}
		}
	}
	FATFS_UnLinkDriver(SDPath);
	for(;;)
	{}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  //LED pogram to be written
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

