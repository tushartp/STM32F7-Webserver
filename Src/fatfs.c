/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
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

#include <stdio.h>
#include <time.h>
#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFs;    /* File system object for SD logical drive */
FIL MyFile;       /* File object for SD */

/* USER CODE BEGIN Variables */
static void SD_Initialize(void);
static void FATFS_SDDiskInit(char *path);
static void FS_FileOperations(void);

static uint8_t isInitialized = 0;
static uint8_t isFsCreated = 0;
//static __IO uint8_t statusChanged = 0;

uint8_t workBuffer[2 * _MAX_SS];
/*
 * ensure that the read buffer 'rtext' is 32-Byte aligned in address and size
 * to guarantee that any other data in the cache won't be affected when the 'rtext'
 * is being invalidated.
 */
ALIGN_32BYTES(uint8_t rtext[64]);
/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
  /* additional user code for init */     
  if(retSD == 0)
  {
	SD_Initialize();
	//osThreadDef(uSDThread, FS_AppThread, osPriorityNormal, 0, 8 * configMINIMAL_STACK_SIZE);
	//osThreadCreate(osThread(uSDThread), NULL);
	FATFS_SDDiskInit(SDPath);
	/* Create Storage Message Queue */
	//osMessageQDef(osqueue, 10, uint16_t);
	//ConnectionEvent = osMessageCreate (osMessageQ(osqueue), NULL);
	//if(isFsCreated)
	//{
		//FS_FileOperations();
	//}

  }
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
	    time_t rawtime;
	    time(&rawtime);
	    struct tm *ptm = localtime(&rawtime);
	    return (DWORD)(ptm->tm_year - 80) << 25
	           | (DWORD)(ptm->tm_mon + 1) << 21
	           | (DWORD)(ptm->tm_mday) << 16
	           | (DWORD)(ptm->tm_hour) << 11
	           | (DWORD)(ptm->tm_min) << 5
	           | (DWORD)(ptm->tm_sec / 2);
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
static void SD_Initialize(void)
{
  if (isInitialized == 0)
  {
	BSP_SD_Init();
    BSP_SD_ITConfig();
     
    if(BSP_SD_IsDetected())
    {
      isInitialized = 1;
    }
  }
}
 static void FATFS_SDDiskInit( char *path )
{
	  /* Register the file system object to the FatFs module */
	  if(f_mount(&SDFatFs, (TCHAR const*)path, 1) == FR_OK)
	  {
		  if (SDFatFs.fs_type == FS_FAT32)
			  isFsCreated = 1;
	  }

	    /* check whether the FS has been already created */
	    if (isFsCreated == 0)
	    {
	      if(f_mkfs(SDPath, FM_FAT32, 0, workBuffer, sizeof(workBuffer)) != FR_OK)
	      {
	        BSP_LED_On(LED_RED);
	        return;
	      }
	      isFsCreated = 1;
	    }
}    

// #if 0
 static void FS_FileOperations(void)
 {
   FILE *fil;
   FRESULT res;                                          /* FatFs function common result code */
   size_t byteswritten, bytesread;                     /* File write/read counts */
   uint8_t wtext[] = "This is STM32 working with FatFs uSD + FreeRTOS CubeMX"; /* File write buffer */

   /* Register the file system object to the FatFs module */
   if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 1) == FR_OK)
   {
		  if (SDFatFs.fs_type == FS_FAT32)
			  isFsCreated = 1;
     /* check whether the FS has been already created */
     if (isFsCreated == 0)
     {
       if(f_mkfs(SDPath, FM_FAT32, 0, workBuffer, sizeof(workBuffer)) != FR_OK)
       {
         BSP_LED_On(LED_RED);
         return;
       }
       isFsCreated = 1;
     }
     fil = fopen("STM321.txt","w+");
     if(fil)
     {
     	vLoggingPrintf("The file is now opened\n");
     	byteswritten = fwrite(wtext, sizeof(uint8_t), sizeof(wtext),fil);
     	if(byteswritten > 0)
     	{
     		vLoggingPrintf("The file is now closed with %d bytes written\n",byteswritten);
 			fclose(fil);

 			fil = fopen("STM321.txt","r");
 			if(fil)
 			{
 				bytesread = fread(rtext, sizeof(uint8_t), sizeof(rtext),fil);
 				vLoggingPrintf("The file is now opened with %d bytes read\n",bytesread);

 				if(bytesread > 0)
 				{
 					fclose(fil);
 		            /* Compare read data with the expected data */
 		            if((bytesread == byteswritten))
 		            {
 		              /* Success of the demo: no error occurrence */
 		              vLoggingPrintf("The read data length and write data length matches\n");
 		              BSP_LED_On(LED_GREEN);
 		              return;
 		            }
 				}
 			}

     	}
     }
   }
 	fclose(fil);
   /* Error */
   BSP_LED_On(LED_RED);
 }
// #endif

#if 0
static void FS_FileOperations(void)
{
  FRESULT res;                                          /* FatFs function common result code */
  uint32_t byteswritten, bytesread;                     /* File write/read counts */
  uint8_t wtext[] = "This is STM32 working with FatFs uSD + FreeRTOS CubeMX"; /* File write buffer */

  /* Register the file system object to the FatFs module */
  if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 1) == FR_OK)
  {
	  if (SDFatFs.fs_type == FS_FAT32)
		  isFsCreated = 1;
    /* check whether the FS has been already created */
    if (isFsCreated == 0)
    {
    	res = f_mkfs(SDPath, FM_FAT32, 0, workBuffer, sizeof(workBuffer));
      if(res != FR_OK)
      {
        BSP_LED_On(LED_RED);
        return;
      }
      isFsCreated = 1;
    }
    /* Create and Open a new text file object with write access */
    if(f_open(&MyFile, "STM321.TXT", FA_CREATE_ALWAYS | FA_WRITE) == FR_OK)
    {
      /* Write data to the text file */
      res = f_write(&MyFile, wtext, sizeof(wtext), (void *)&byteswritten);

      if((byteswritten > 0) && (res == FR_OK))
      {
        /* Close the open text file */
        f_close(&MyFile);

        /* Open the text file object with read access */
        if(f_open(&MyFile, "STM321.TXT", FA_READ) == FR_OK)
        {
          /* Read data from the text file */
          res = f_read(&MyFile, rtext, sizeof(rtext), (void *)&bytesread);

          if((bytesread > 0) && (res == FR_OK))
          {
            /* Close the open text file */
            f_close(&MyFile);

            /* Compare read data with the expected data */
            if((bytesread == byteswritten))
            {
              /* Success of the demo: no error occurrence */
              BSP_LED_On(LED_GREEN);
              return;
            }
          }
        }
      }
    }
  }
  /* Error */
  BSP_LED_On(LED_RED);
}
#endif

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
