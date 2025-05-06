/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v2.0_Cube
  * @brief          : Memory management layer.
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
#include "usbd_storage_if.h"

/* USER CODE BEGIN INCLUDE */

#include <string.h>

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */



/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */



/* USER CODE BEGIN PRIVATE_DEFINES */

#define STORAGE_FLASH_SIZE        (24 * 1024)              			// buffer
#define STORAGE_FLASH_PAGE_SIZE   0x400                         	// 1 KB per flash page

__attribute__((section(".storage_data"))) const uint8_t storage_buffer[STORAGE_FLASH_SIZE] = {
    // Fill with 0xFF
    [0 ... (STORAGE_FLASH_SIZE - 1)] = 0xFF
};

#define STORAGE_FLASH_START_ADDR  (uint32_t)storage_buffer

#define STORAGE_LUN_NBR           1
#define STORAGE_BLK_SIZ           0x200        // 512 bytes (MSC standard block)
#define STORAGE_BLK_NBR           (STORAGE_FLASH_SIZE / STORAGE_BLK_SIZ)  // 48 blocks

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA_FS */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata_FS[] = {/* 36 */

  /* LUN 0 */
  0x00,
  0x80,
  0x02,
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,
  0x00,
  'I', 'F', 'R', 'S', '-', 'C', 'A', 'N', /* Manufacturer : 8 bytes */
  'S', 'D', 't', 'o', 'U', 'S', 'B', ' ', /* Product      : 16 Bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '0' ,'1'                      /* Version      : 4 Bytes */
};
/* USER CODE END INQUIRY_DATA_FS */

/* USER CODE BEGIN PRIVATE_VARIABLES */

volatile uint8_t flash_busy = 0;

struct write_buffer {
	volatile uint8_t buffer[STORAGE_BLK_SIZ];
	volatile uint8_t written;
	volatile uint8_t* flash_block_start;
};

write_buffer _buffer_write[4];

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init_FS(uint8_t lun);
static int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady_FS(uint8_t lun);
static int8_t STORAGE_IsWriteProtected_FS(uint8_t lun);
static int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun_FS(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

HAL_StatusTypeDef write_flash_buffer(write_buffer buffer[4]) {
	for (uint8_t i = 0; i < 4; i++) {
		if (buffer[i].written == 1) {
			uint32_t addr = STORAGE_FLASH_START_ADDR + blk_addr * STORAGE_BLK_SIZ;
			uint32_t total_bytes = blk_len * STORAGE_BLK_SIZ;

			if ((addr + total_bytes) > (STORAGE_FLASH_START_ADDR + STORAGE_FLASH_SIZE))
				return USBD_FAIL;

			HAL_FLASH_Unlock();

			// Erase needed pages
			uint32_t start_page = addr & ~(STORAGE_FLASH_PAGE_SIZE - 1);
			uint32_t end_addr = addr + total_bytes;

			while (start_page < end_addr) {
				FLASH_EraseInitTypeDef erase = {
					.TypeErase = FLASH_TYPEERASE_PAGES,
					.PageAddress = start_page,
					.NbPages = 1
				};
				uint32_t page_error;
				if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
					HAL_FLASH_Lock();
					return USBD_FAIL;
				}
				start_page += STORAGE_FLASH_PAGE_SIZE;
			}

			// Write half-words
			for (uint32_t i = 0; i < total_bytes; i += 2) {
				uint16_t data = buf[i] | (buf[i + 1] << 8);
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, addr + i, data) != HAL_OK) {
					HAL_FLASH_Lock();
					return USBD_FAIL;
				}
			}

			HAL_FLASH_Lock();
		}
	}
	return HAL_OK;
}

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops_FS =
{
  STORAGE_Init_FS,
  STORAGE_GetCapacity_FS,
  STORAGE_IsReady_FS,
  STORAGE_IsWriteProtected_FS,
  STORAGE_Read_FS,
  STORAGE_Write_FS,
  STORAGE_GetMaxLun_FS,
  (int8_t *)STORAGE_Inquirydata_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes over USB FS IP
  * @param  lun:
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Init_FS(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  .
  * @param  lun: .
  * @param  block_num: .
  * @param  block_size: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_GetCapacity_FS(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
    *block_num = STORAGE_FLASH_SIZE / STORAGE_BLK_SIZ;  // 64 blocks
    *block_size = STORAGE_BLK_SIZ;                      // 512 bytes
    return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsReady_FS(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
	return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_IsWriteProtected_FS(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 6 */
    uint32_t addr = STORAGE_FLASH_START_ADDR + blk_addr * STORAGE_BLK_SIZ;
    uint32_t total_bytes = blk_len * STORAGE_BLK_SIZ;

    if ((addr + total_bytes) > (STORAGE_FLASH_START_ADDR + STORAGE_FLASH_SIZE))
        return USBD_FAIL;

    memcpy(buf, (const void *)addr, total_bytes);
	return (USBD_OK);
  /* USER CODE END 6 */
}

/**
   * @brief  .
  * @param  lun: .
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write_FS(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
	flash_busy = 1;

	// Logic w/ buffer

	flash_busy = 0;
	return (USBD_OK);
  /* USER CODE END 7 */
}

/**
  * @brief  .
  * @param  None
  * @retval .
  */
int8_t STORAGE_GetMaxLun_FS(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

