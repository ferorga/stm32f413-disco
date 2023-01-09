#include "stm32f4xx.h"
#include "clock.h"
#include "register.h"

/*******************************************************
 ***          local defines and data types           ***
 *******************************************************/
static volatile T_HWREG * const CLOCK_RCC_AHB1ENR = (T_HWREG*)REGISTER_RCC_AHB1ENR_BASE_ADDRESS;     ///< RCC_AHB1ENR base address
static volatile T_HWREG * const CLOCK_RCC_APB1ENR = (T_HWREG*)REGISTER_RCC_APB1ENR_BASE_ADDRESS;     ///< RCC_APB1ENR base address
static volatile T_HWREG * const CLOCK_RCC_APB2ENR = (T_HWREG*)REGISTER_RCC_APB2ENR_BASE_ADDRESS;     ///< RCC_APB2ENR base address
static volatile T_HWREG * const CLOCK_RCC_AHB3ENR = (T_HWREG*)REGISTER_RCC_AHB3ENR_BASE_ADDRESS;     ///< RCC_AHB3ENR base address

void clock_SystInit(void)
{
    static volatile T_HWREG * const CLOCK_RCC_CR     = (T_HWREG*)REGISTER_RCC_CR_BASE_ADDRESS;          ///< RCC_CR base address
    static volatile T_HWREG * const CLOCK_RCC_PLLCFG = (T_HWREG*)REGISTER_RCC_PLLCFGR_BASE_ADDRESS;     ///< RCC_PLLCFG base address
    static volatile T_HWREG * const CLOCK_RCC_CFGR   = (T_HWREG*)REGISTER_RCC_CFGR_BASE_ADDRESS;        ///< RCC_CFGR base address
    static volatile T_HWREG * const CLOCK_RCC_PWR    = (T_HWREG*)REGISTER_RCC_PWR_BASE_ADDRESS;         ///< RCC_PWR base address
    static volatile T_HWREG * const CLOCK_FLASH_ACR  = (T_HWREG*)REGISTER_FLASH_ACR_BASE_ADDRESS;       ///< FLASH_ACR base address

    CLOCK_RCC_CR->ulHWREG      = 0x00000081;
    CLOCK_RCC_PLLCFG->ulHWREG  = 0x24003010;    
    CLOCK_RCC_APB1ENR->ulHWREG = 0x00000400;
    CLOCK_RCC_CFGR->ulHWREG    = 0x00000000;
    
    uint32_t ulCnt = (uint32_t)0U;

    //Enable the system configuration controller clock
    CLOCK_RCC_APB2ENR->ulHWREG |= REGISTER_RCC_APB2_SYSCFG_EN;

    //Configure the main internal regulator output voltage
    CLOCK_RCC_AHB1ENR->ulHWREG |= REGISTER_RCC_APB1ENR_PWR_EN;
    
    CLOCK_RCC_APB1ENR->ulHWREG |= (1 << 4); //TIM 6

    //
    CLOCK_RCC_PWR->ulHWREG |= REGISTER_RCC_PWR_VOS_3;

    //Enable HSEbypass
    //CLOCK_RCC_CR->ulHWREG |= REGISTER_RCC_CR_PLLBYPASS;

    //Enable the HSE
    CLOCK_RCC_CR->ulHWREG |= REGISTER_RCC_CR_HSEON;

    /*while((CLOCK_RCC_CR->ulHWREG & REGISTER_RCC_CR_HSERDY) != REGISTER_RCC_CR_HSERDY)
    {
        ulCnt++;
    }*/

    //Clear the RCC_PLLCFG Register
    CLOCK_RCC_PLLCFG->ulHWREG = ~REGISTER_REG32_ALL_BIT;

    //Configure the system clock to work with the PLL
    /*CLOCK_RCC_PLLCFG->ulHWREG |= (REGISTER_RCC_PLLCFGR_PLLM  |
                            REGISTER_RCC_PLLCFGR_PLLN  |
                            REGISTER_RCC_PLLCFG_PLLSRC |
                            REGISTER_RCC_PLLCFG_PLLQ   |
                            REGISTER_RCC_PLLCFG_PLLR);*/
    CLOCK_RCC_PLLCFG->ulHWREG |= (10 << 0)  |
                                 (250 << 6) |
                                 (1 << 16)  |
                                 (0 << 22)  |
                                 (2 << 24)  |
                                 (2 << 28);

    //start PLL
    CLOCK_RCC_CR->ulHWREG |= (1 << 0)  | //HSI ON
                             (0 << 16) | //HSE ON
                             (1 << 24) | //PLL ON
                             (0 << 26); //PLL2

    //configure the Flash
    CLOCK_FLASH_ACR->ulHWREG |= (REGISTER_FLASH_ACR_PRFTEN |
                           REGISTER_FLASH_ACR_ICEN   |
                           REGISTER_FLASH_ACR_DCEN   |
                           REGISTER_FLASH_ACR_LATENCY);

    //While until the PLL is ready
    while((CLOCK_RCC_CR->ulHWREG & REGISTER_RCC_CR_PLLONRDY) != REGISTER_RCC_CR_PLLONRDY)
    {
        if(ulCnt == REGISTER_REG32_ALL_BIT)
        {
            ulCnt = 0x00;
        }
        ulCnt++;
    }
    //Set the APB1 prescaler
    CLOCK_RCC_CFGR->ulHWREG |= REGISTER_RCC_CFGR_PPRE_APB1_2;

    //chosse PLL as systemclock
    CLOCK_RCC_CFGR->ulHWREG |= REGISTER_RCC_CFGR_SW_PLL;

    //wait until
    while((CLOCK_RCC_CFGR->ulHWREG & REGISTER_RCC_CFGR_SWS_PLL) != REGISTER_RCC_CFGR_SWS_PLL)
    {
        if(ulCnt == REGISTER_REG32_ALL_BIT)
        {
            ulCnt = 0x00;
        }
        ulCnt++;
    }
}

void clock_PortInit(void)
{
        //Enable the port clock
    CLOCK_RCC_AHB1ENR->ulHWREG |= (REGISTER_RCC_AHB1ENR_DMA1_EN  |      //Enable DMA1 Clock
                                   REGISTER_RCC_AHB1ENR_DMA2_EN  |      //Enable DMA2 Clock
                                   REGISTER_RCC_AHB1ENR_GPIOG_EN |      //Enable clock for Port G
                                   REGISTER_RCC_AHB1ENR_GPIOH_EN |      //Enable clock for Port H
                                   REGISTER_RCC_AHB1ENR_GPIOF_EN |      //Enable clock for Port F
                                   REGISTER_RCC_AHB1ENR_GPIOE_EN |      //Enable clock for Port E
                                   REGISTER_RCC_AHB1ENR_GPIOD_EN |      //Enable clock for Port D
                                   REGISTER_RCC_AHB1ENR_GPIOC_EN |      //Enable clock for Port C
                                   REGISTER_RCC_AHB1ENR_GPIOB_EN |      //Enable clock for Port B
                                   REGISTER_RCC_AHB1ENR_GPIOA_EN);      //Enable clock for Port A
}

void clock_DeInit(void)
{
    static volatile T_HWREG * const APB1RST = (T_HWREG*)REGISTER_RCC_APB1RSTR_BASE_ADDRESS;
    static volatile T_HWREG * const APB2RST = (T_HWREG*)REGISTER_RCC_APB2RSTR_BASE_ADDRESS;
    static volatile T_HWREG * const AHB1RST = (T_HWREG*)REGISTER_RCC_AHB1RSTR_BASE_ADDRESS;
    static volatile T_HWREG * const AHB2RST = (T_HWREG*)REGISTER_RCC_AHB2RSTR_BASE_ADDRESS;
    static volatile T_HWREG * const AHB3RST = (T_HWREG*)REGISTER_RCC_AHB3RSTR_BASE_ADDRESS;
    static volatile T_HWREG * const RCC_CR  = (T_HWREG*)REGISTER_RCC_CR_BASE_ADDRESS;
    static volatile T_HWREG * const RCC_CFGR= (T_HWREG*)REGISTER_RCC_CFGR_BASE_ADDRESS;
    static volatile T_HWREG * const RCC_CIR = (T_HWREG*)REGISTER_RCC_CIR_BASE_ADDRESS; 
    static volatile T_HWREG * const RCC_CSR = (T_HWREG*)REGISTER_RCC_CSR_BASE_ADDRESS; 
    
    APB1RST->ulHWREG = 0xFFFFFFFFU;
    APB1RST->ulHWREG = 0x00000000U;
    APB2RST->ulHWREG = 0xFFFFFFFFU;
    APB2RST->ulHWREG = 0x00000000U;
    AHB1RST->ulHWREG = 0xFFFFFFFFU;
    AHB1RST->ulHWREG = 0x00000000U;
    AHB2RST->ulHWREG = 0xFFFFFFFFU;
    AHB2RST->ulHWREG = 0x00000000U;
    AHB3RST->ulHWREG = 0xFFFFFFFFU;
    AHB3RST->ulHWREG = 0x00000000U;
    
    RCC_CR->ulHWREG |= REGISTER_RCC_CR_HSION;
    while((RCC_CR->ulHWREG & REGISTER_RCC_CR_HSIRDY) != REGISTER_RCC_CR_HSIRDY);
    
    RCC_CR->ulHWREG |= REGISTER_RCC_CR_HSITRIM_4;
    
    RCC_CFGR->ulHWREG = 0x0U;
    
    while((RCC_CFGR->ulHWREG & REGISTER_RCC_CFGR_SWS) == REGISTER_RCC_CFGR_SWS);
    
    RCC_CR->ulHWREG =~ REGISTER_RCC_CR_HSEON;
    RCC_CR->ulHWREG =~ REGISTER_RCC_CR_HSEBYP;
    RCC_CR->ulHWREG =~ REGISTER_RCC_CR_CSSON;
    
    while((RCC_CR->ulHWREG & REGISTER_RCC_CR_HSERDY) != REGISTER_RCC_CR_HSERDY);
    
    RCC_CR->ulHWREG = ~REGISTER_RCC_CR_PLLON;
        
    RCC_CIR->ulHWREG = 0x0U;
    
    RCC_CSR->ulHWREG = 0x0E000000;
 }

/**
 \brief Function to set the CLOCK_RCC Registers
 \param[in]     tRegSelect      RCC Register
 \param[in]     ulValue         Value which is to be set
*/
void clock_SetRCCRegister(T_CLOCK_RCC tRegSelect, uint32_t ulValue)
{
    switch(tRegSelect)
    {
        case RCC_AHB1:
        {
            CLOCK_RCC_AHB1ENR->ulHWREG |= ulValue;
            break;
        }
        case RCC_AHB3:
        {
            CLOCK_RCC_AHB3ENR->ulHWREG |= ulValue;
            break;
        }
        case RCC_APB1:
        {
            CLOCK_RCC_APB1ENR->ulHWREG |= ulValue;
            break;
        }
        case RCC_APB2:
        {
            CLOCK_RCC_APB2ENR->ulHWREG |= ulValue;
            break;
        }
        default:
        {
            /* Intentionally Empty */
            break;
        }
    }
}


/**
 \brief Function to get the CLOCK_RCC Registers
 \param[in]     tRegSelect      RCC Register
 \return        Pointer to the RCC Register
*/
volatile T_HWREG* clock_GetRCCRegister(T_CLOCK_RCC tRegSelect)
{
    volatile T_HWREG* ptRegister = NULL_PTR;
    switch(tRegSelect)
    {
        case RCC_AHB1:
        {
            ptRegister = CLOCK_RCC_AHB1ENR;
            break;
        }
        case RCC_AHB3:
        {
            ptRegister = CLOCK_RCC_AHB3ENR;
            break;
        }
        case RCC_APB1:
        {
            ptRegister = CLOCK_RCC_APB1ENR;
            break;
        }
        case RCC_APB2:
        {
            ptRegister = CLOCK_RCC_APB2ENR;
            break;
        }
        default:
        {
            /* Intentionally Empty */
            break;
        }
    }
    return ptRegister;
}

void clock_Init(void)
{
    clock_SystInit();
    clock_PortInit();
}