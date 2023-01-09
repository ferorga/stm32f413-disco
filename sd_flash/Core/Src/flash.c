/**
  ******************************************************************************
  * @file    IAP_Main/Src/flash_if.c 
  * @author  MCD Application Team
  * @version 1.0.0
  * @date    8-April-2015
  * @brief   This file provides all the memory related operation functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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

/** @addtogroup STM32L4xx_IAP
  * @{
  */

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "flash.h"
//#include "clock.h"
#include "appError.h"

#include "stm32f4xx_hal_flash.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/*  1M5 flash 1.5 * 1024 * 1024 */
#define FLASH_START_ADRESS    	0x08000000
#define FLASH_SECTOR_SIZE	  	0x20000		// the application is located in 128kByte sectors
#define FLASH_FIRST_APP_SECTOR	5			// number of to first 128kByte sector used for the application

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

bool_t flash_IsLocked(void)
{
    volatile T_HWREG * const FLASH_CR = (T_HWREG*)REGISTER_FLASH_CR_BASE_ADDRESS;
    return ((FLASH_CR->ulHWREG & REGISTER_FLASH_CR_LOCK_MSK) == REGISTER_FLASH_CR_LOCKED);
}

bool_t flash_IsBusy(void)
{
    volatile T_HWREG * const FLASH_SR = (T_HWREG*)REGISTER_FLASH_SR_BASE_ADDRESS;
    
    return (FLASH_SR->ulHWREG & REGISTER_FLASH_SR_BSY_MSK) == REGISTER_FLASH_SR_BSY;
}

uint32_t flash_Unlock(void)
{
    volatile T_HWREG * const FLASH_KEYR = (T_HWREG*)REGISTER_FLASH_KEYR_BASE_ADDRESS;
    volatile T_HWREG * const FLASH_CR = (T_HWREG*)REGISTER_FLASH_CR_BASE_ADDRESS;
    
    uint32_t ulResult = GENERIC_ERROR;
    
    if (flash_IsLocked() == TRUE)
    {
        FLASH_KEYR->ulHWREG = REGISTER_FLASH_KEY1;
        FLASH_KEYR->ulHWREG = REGISTER_FLASH_KEY2;
        if ((FLASH_CR->ulHWREG & REGISTER_FLASH_CR_LOCK_MSK) == REGISTER_FLASH_CR_NOT_LOCKED)
        {
            ulResult = SUCCESS;
        }
    }
    
    return ulResult;
}

uint32_t flash_Lock(void)
{
    volatile T_HWREG * const FLASH_CR = (T_HWREG*)REGISTER_FLASH_CR_BASE_ADDRESS;
    
    FLASH_CR->ulHWREG |= REGISTER_FLASH_CR_LOCKED;
    
    return SUCCESS;
} 

void flash_Init(void)
{
    volatile T_HWREG * const FLASH_SR = (T_HWREG*)REGISTER_FLASH_SR_BASE_ADDRESS;
    
    flash_Unlock();
    
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    
    static volatile T_HWREG * const CLOCK_FLASH_ACR  = (T_HWREG*)REGISTER_FLASH_ACR_BASE_ADDRESS;       ///< FLASH_ACR base address
    FLASH_SR->ulHWREG = REGISTER_FLASH_SR_RESET;
    
    CLOCK_FLASH_ACR->ulHWREG |= (REGISTER_FLASH_ACR_PRFTEN |
                       REGISTER_FLASH_ACR_ICEN   |
                       REGISTER_FLASH_ACR_DCEN   |
                       REGISTER_FLASH_ACR_LATENCY);
    
    flash_Lock();
}

void flash_FlushCaches(void)
{
    volatile T_HWREG * const FLASH_ACR = (T_HWREG*)REGISTER_FLASH_ACR_BASE_ADDRESS;
    /* Instruction Cache */
    if ((FLASH_ACR->ulHWREG & REGISTER_FLASH_ACR_ICEN) == REGISTER_FLASH_ACR_ICEN)
    {
        /* Disable */
        FLASH_ACR->ulHWREG &= ~REGISTER_FLASH_ACR_ICEN;
        /* Reset */
        FLASH_ACR->ulHWREG |= REGISTER_FLASH_ACR_ICRST;
        FLASH_ACR->ulHWREG &= ~REGISTER_FLASH_ACR_ICRST;
        /* Enable */
        FLASH_ACR->ulHWREG |= REGISTER_FLASH_ACR_ICEN;
    }
    /* Data Cache */
    if ((FLASH_ACR->ulHWREG & REGISTER_FLASH_ACR_DCEN) == REGISTER_FLASH_ACR_DCEN)
    {
        /* Disable */
        FLASH_ACR->ulHWREG &= ~REGISTER_FLASH_ACR_DCEN;
        /* Reset */
        FLASH_ACR->ulHWREG |= REGISTER_FLASH_ACR_DCRST;
        FLASH_ACR->ulHWREG &= ~REGISTER_FLASH_ACR_DCRST;
        /* Enable */
        FLASH_ACR->ulHWREG |= REGISTER_FLASH_ACR_DCEN;
    }
}

uint32_t flash_EraseApplication(uint32_t ulLength)
{
    uint32_t ulResult = GENERIC_ERROR;
    volatile T_HWREG * const FLASH_CR = (T_HWREG*)REGISTER_FLASH_CR_BASE_ADDRESS;
    const uint32_t ulNbSectors = (ulLength / FLASH_SECTOR_SIZE) + 1;
    
    /* Unlock */
    if(flash_Unlock() == SUCCESS)        
    {    
        while(flash_IsBusy()){};
        
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    
        for(uint32_t ulIndex = FLASH_FIRST_APP_SECTOR; ulIndex < (ulNbSectors + FLASH_FIRST_APP_SECTOR); ulIndex++)
        {
            /* Erase Sector */
            FLASH_CR->ulHWREG &= ~REGISTER_FLASH_CR_PSIZE;
            FLASH_CR->ulHWREG |= 0x00000200U;
            FLASH_CR->ulHWREG &= ~REGISTER_FLASH_CR_SNB;
            FLASH_CR->ulHWREG |= REGISTER_FLASH_CR_SER | (ulIndex << 3U);
            FLASH_CR->ulHWREG |= REGISTER_FLASH_CR_STRT;            
            
            /* Wait while busy */
            while(flash_IsBusy()){};
            
            /* Clear SER and SNB registers */
            FLASH_CR->ulHWREG &= ~REGISTER_FLASH_CR_SER;
            FLASH_CR->ulHWREG &= ~REGISTER_FLASH_CR_SNB;
        }
        /* Flush Flash Caches */
        flash_FlushCaches();        
        
        /* Lock */
        flash_Lock();
        
        ulResult = SUCCESS;
    }
        
    return ulResult;
}

uint32_t flash_WriteApplication(uint8_t *ulSource, uint32_t ulLength, uint32_t *ulFragOffset)
{
    uint32_t ulResult = GENERIC_ERROR;
    volatile T_HWREG * const FLASH_CR = (T_HWREG*)REGISTER_FLASH_CR_BASE_ADDRESS;    
    volatile uint8_t *ulDst = (uint8_t *)(APPLICATION_ADDRESS + *ulFragOffset);
    uint32_t i = 0;
    
    while(flash_IsBusy()){};
    
    /* Unlock */
    if (flash_Unlock() == SUCCESS)
    {
        for (i = 0; i<(ulLength); i++)
        {
            while(flash_IsBusy()){};
                
            /* Write data into destination */
            FLASH_CR->ulHWREG &= ~REGISTER_FLASH_CR_PSIZE;
            //FLASH_CR->ulHWREG |= 0x200;                
            FLASH_CR->ulHWREG |= REGISTER_FLASH_CR_PG;
            
            *ulDst = ulSource[i];
            
            //if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, APPLICATION_ADDRESS + *ulFragOffset, ulSource[i]) == HAL_OK);
            
            while(flash_IsBusy()){};
            
            FLASH_CR->ulHWREG &= ~REGISTER_FLASH_CR_PG;                               
            
            /* Check the written value */
            if (*(volatile uint8_t*)ulDst != ulSource[i])
            {
                break;
            }
            /* Increment FLASH destination address */                
            ulDst++;
            *ulFragOffset+=1;            
        }
        
        /* Lock */
        flash_Lock();               
        
        ulResult = SUCCESS;
    }       
    
    return ulResult;
}
