/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : mbedtls.c
  * Description        : This file provides code for the configuration
  *                      of the mbedtls instances.
  ******************************************************************************
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
#include "mbedtls.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include "lwip.h"
#include "mbedtls/platform.h"
#include "mbedtls/debug.h"
#include <string.h>
/* USER CODE END 0 */

/* USER CODE BEGIN 1 */
static void CRYPTO_ConfigureHeap( void );
/* USER CODE END 1 */

/* Global variables ---------------------------------------------------------*/

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/* MBEDTLS init function */
void MX_MBEDTLS_Init(void)
{
   /**
  */
  /* USER CODE BEGIN 3 */
	MX_LWIP_Init();
	CRYPTO_ConfigureHeap();
	mbedtls_platform_set_printf(vLoggingPrintf);
  /* USER CODE END 3 */

}

/* USER CODE BEGIN 4 */
/**
 * @brief Implements libc calloc semantics using the FreeRTOS heap
 */
static void * prvCalloc( size_t xNmemb,
                         size_t xSize )
{
    void * pvNew = pvPortMalloc( xNmemb * xSize );

    if( NULL != pvNew )
    {
        memset( pvNew, 0, xNmemb * xSize );
    }

    return pvNew;
}

/**
 * @brief Overrides CRT heap callouts to use FreeRTOS instead
 */
static void CRYPTO_ConfigureHeap( void )
{
    /*
     * Ensure that the FreeRTOS heap is used
     */
    mbedtls_platform_set_calloc_free( prvCalloc, vPortFree ); /*lint !e534 This function always return 0. */
}
/* USER CODE END 4 */

/**
  * @}
  */
 
/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
