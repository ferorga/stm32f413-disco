/**
 \file      register.h
 \author    mko, KAPPA opto-electronics GmbH
 \date      01.07.2022
 \details   HAL(Hardware Abstraction Layer) registers used from microcontroller peripherals.
*/

#ifndef __REGISTER_H__
#define __REGISTER_H__

#include "types.h"

/**/
#define REGISTER_REG32_ALL_BIT  (uint32_t)0xFFFFFFFFUL          ///< 32-bit register all bits

/*---Define for RCC AHB1 clock enable register --*/
#define REGISTER_RCC_AHB1ENR_GPIOA_EN    (uint32_t)0x00000001UL     ///< RCC AHB1 clock enable register GPIOA enable
#define REGISTER_RCC_AHB1ENR_GPIOB_EN    (uint32_t)0x00000002UL     ///< RCC AHB1 clock enable register GPIOB enable
#define REGISTER_RCC_AHB1ENR_GPIOC_EN    (uint32_t)0x00000004UL     ///< RCC AHB1 clock enable register GPIOC enable
#define REGISTER_RCC_AHB1ENR_GPIOD_EN    (uint32_t)0x00000008UL     ///< RCC AHB1 clock enable register GPIOD enable
#define REGISTER_RCC_AHB1ENR_GPIOE_EN    (uint32_t)0x00000010UL     ///< RCC AHB1 clock enable register GPIOE enable
#define REGISTER_RCC_AHB1ENR_GPIOF_EN    (uint32_t)0x00000020UL     ///< RCC AHB1 clock enable register GPIOF enable
#define REGISTER_RCC_AHB1ENR_GPIOH_EN    (uint32_t)0x00000001UL     ///< RCC AHB1 clock enable register GPIOH enable
#define REGISTER_RCC_AHB1ENR_GPIOG_EN    (uint32_t)0x00000040UL     ///< RCC AHB1 clock enable register GPIOG enable
#define REGISTER_RCC_AHB1ENR_CRC_EN      (uint32_t)0x00001000UL     ///< RCC AHB1 clock enable register CRC enable
#define REGISTER_RCC_AHB1ENR_DMA1_EN     (uint32_t)0x00200000UL     ///< RCC AHB1 clock enable register DMA1 enable
#define REGISTER_RCC_AHB1ENR_DMA2_EN     (uint32_t)0x00400000UL     ///< RCC AHB1 clock enable register DMA2 enable

/**---Define for RCC APB1 clock enable Register */
#define REGISTER_RCC_APB1ENR_UART5_EN    (uint32_t)0x00100000UL     ///< RCC APB1 clock enable register UART5 enable
#define REGISTER_RCC_APB1ENR_UART4_EN    (uint32_t)0x00080000UL     ///< RCC APB1 clock enable register UART4 enable
#define REGISTER_RCC_APB1ENR_UART7_EN    (uint32_t)0x40000000UL     ///< RCC APB1 clock enable register UART7 enable
#define REGISTER_RCC_APB1ENR_I2C1_EN     (uint32_t)0x00200000UL     ///< RCC APB1 clock enable register I2C1 enable
#define REGISTER_RCC_APB1ENR_I2C2_EN     (uint32_t)0x00400000UL     ///< RCC APB1 clock enable register I2C2 enable
#define REGISTER_RCC_APB1ENR_I2C3_EN     (uint32_t)0x00800000UL     ///< RCC APB1 clock enable register I2C3 enable
#define REGISTER_RCC_APB1ENR_PWR_EN      (uint32_t)0x10000000UL     ///< RCC APB1 clock enable register PWR enable

/**---Define for RCC APB1 clock enable Register */
#define REGISTER_RCC_APB2ENR_ADC_EN      (uint32_t)0x00000100UL     ///< RCC APB1 clock enable register ADC enable
#define REGISTER_RCC_APB2ENR_UART10_EN   (uint32_t)0x00000080UL     ///< RCC APB1 clock enable register UART10 enable
#define REGISTER_RCC_APB2ENR_UART6_EN    (uint32_t)0x00000020UL     ///< RCC APB1 clock enable register UART6 enable

/*--Define Base ADDRESS RCC AHB1 Clock enable Register */
#define REGISTER_RCC_AHB1ENR_BASE_ADDRESS 0x40023830UL      ///< RCC AHB1 clock enable register base address

/*---Define Base Address RCC APB1 clock enable    */
#define REGISTER_RCC_APB1ENR_BASE_ADDRESS (uint32_t)0x40023840UL        ///< RCC APB1 clock enable register base address

/*---Define Base Address RCC APB1 clock enable    */
#define REGISTER_RCC_APB2ENR_BASE_ADDRESS (uint32_t)0x40023844UL        ///< RCC APB1 clock enable register base address
#define REGISTER_RCC_APB2_SYSCFG_EN       (uint32_t)0x00004000UL        ///< RCC APB1 clock enable register SYSCFG enable
#define REGISTER_RCC_APB2ENR_SPI5_EN                0x00100000UL        ///< RCC APB1 clock enable register SPI5 enable

#define REGISTER_RCC_APB1RSTR_BASE_ADDRESS (uint32_t)0x40023820UL
#define REGISTER_RCC_APB2RSTR_BASE_ADDRESS (uint32_t)0x40023824UL
#define REGISTER_RCC_AHB1RSTR_BASE_ADDRESS (uint32_t)0x40023810UL
#define REGISTER_RCC_AHB2RSTR_BASE_ADDRESS (uint32_t)0x40023814UL
#define REGISTER_RCC_AHB3RSTR_BASE_ADDRESS (uint32_t)0x40023818UL

/*define Base Adresse RCC_AHB3ENR*/
#define REGISTER_RCC_AHB3ENR_BASE_ADDRESS (uint32_t)0x40023838UL        ///< RCC AHB3ENR base address

/*define RCC_CR base Addresse*/
#define REGISTER_RCC_CR_BASE_ADDRESS      (uint32_t)0x40023800UL        ///< RCC register base address
#define REGISTER_RCC_CR_HSION             (uint32_t)0x00000001UL        ///< RCC register Internal high-speed clock enable
#define REGISTER_RCC_CR_HSITRIM_0         (uint32_t)0x00000008UL
#define REGISTER_RCC_CR_HSITRIM_1         (uint32_t)0x00000010UL
#define REGISTER_RCC_CR_HSITRIM_2         (uint32_t)0x00000020UL
#define REGISTER_RCC_CR_HSITRIM_3         (uint32_t)0x00000040UL
#define REGISTER_RCC_CR_HSITRIM_4         (uint32_t)0x00000080UL
#define REGISTER_RCC_CR_HSEBYP            (uint32_t)0x00040000UL
#define REGISTER_RCC_CR_CSSON             (uint32_t)0x00080000UL
#define REGISTER_RCC_CR_HSEON             (uint32_t)0x00010000UL        ///< RCC register HSE clock enable
#define REGISTER_RCC_CR_HSERDY            (uint32_t)0x00020000UL        ///< RCC register HSE clock ready flag
#define REGISTER_RCC_CR_PLLON             (uint32_t)0x01000000UL        ///< RCC register Main PLL clock enable
#define REGISTER_RCC_CR_PLLONRDY          (uint32_t)0x02000000UL        ///< RCC register Main PLL clock ready flag
#define REGISTER_RCC_CR_PLLBYPASS         (uint32_t)0x00040000UL        ///< RCC register HSE clock bypass
#define REGISTER_RCC_CR_HSIRDY            (uint32_t)0x00000002UL        ///< RCC register Internal high-speed clock ready flag

#define REGISTER_RCC_PLLCFGR_BASE_ADDRESS (uint32_t)0x40023804UL        ///< RCC PLL configuration register base address

/* SYSCLK = Oscillator / PLLM * PLLN / PLLP */
#define REGISTER_RCC_PLLCFGR_PLLM         (uint32_t)0x00000005UL        ///< PLLM = 5
#define REGISTER_RCC_PLLCFGR_PLLN         (uint32_t)0x00001900UL        ///< PLLN = 100
                                                                /* PLLP = 2 (default)*/
#define REGISTER_RCC_PLLCFG_PLLSRC        (uint32_t)0x00400000UL        ///< HSE as PLL source
#define REGISTER_RCC_PLLCFG_PLLQ          (uint32_t)0x02000000UL        ///< PLLQ = 2
#define REGISTER_RCC_PLLCFG_PLLR          (uint32_t)0x20000000UL        ///< PLLR = 2

/**/
#define REGISTER_RCC_CFGR_BASE_ADDRESS    (uint32_t)0x40023808UL        ///< RCC clock configuration register base address
#define REGISTER_RCC_CFGR_SWS_PLL         (uint32_t)0x00000008UL
#define REGISTER_RCC_CFGR_SWS_0           (uint32_t)0x00000004UL        ///< RCC clock configuration register system clock switch status
#define REGISTER_RCC_CFGR_SWS_1           (uint32_t)0x00000008UL        ///< RCC clock configuration register system clock switch status
#define REGISTER_RCC_CFGR_SWS             (uint32_t)0x0000000CUL        ///< RCC clock configuration register system clock switch status
#define REGISTER_RCC_CFGR_SW_PLL          (uint32_t)0x00000002UL        ///< RCC clock configuration register system clock switch
#define REGISTER_RCC_CFGR_SW_0            (uint32_t)0x00000001UL        ///< RCC clock configuration register system clock switch
#define REGISTER_RCC_CFGR_SW_1            (uint32_t)0x00000002UL        ///< RCC clock configuration register system clock switch
#define REGISTER_RCC_CFGR_SW              (uint32_t)0x00000003UL        ///< RCC clock configuration register system clock switch
#define REGISTER_RCC_CFGR_HPRE            (uint32_t)0x0000000000        ///< RCC clock configuration register AHB prescaler
#define REGISTER_RCC_CFGR_PPRE_APB1_2     (uint32_t)0x00001000UL        ///< RCC clock configuration register APB low speed prescaler (APB1)

#define REGISTER_RCC_CIR_BASE_ADDRESS     (uint32_t)0x4002380CUL

#define REGISTER_RCC_CSR_BASE_ADDRESS     (uint32_t)0x40023874UL

 /**/
#define REGISTER_RCC_PWR_BASE_ADDRESS     (uint32_t)0x40007000UL        ///< RCC PWR register base address
#define REGISTER_RCC_PWR_VOS_3            (uint32_t)0x0000C000UL        ///< RCC PWR register vos 3

/*Define DMA1 Base Adresse */
#define  REGISTER_DMA1_BASE_ADDRESS       (uint32_t)0x40026000UL        ///< DMA1 base address

/*Define DMA1 Base Adresse */
#define REGISTER_DMA2_BASE_ADDRESS        (uint32_t)0x40026400UL        ///< DMA2 base address
#define REGISTER_DMA_HIFCR_CTCIF4         (uint32_t)0x0000003FUL        ///< DMA high interrupt flag clear register stream 4 clear transfer complete interrupt flag
#define REGISTER_DMA_HIFCR_CTCIF5         (uint32_t)0x00000F00UL        ///< DMA high interrupt flag clear register stream 5 clear transfer complete interrupt flag
#define REGISTER_DMA_HIFCR_CTCIF7         (uint32_t)0x0F000000UL        ///< DMA high interrupt flag clear register stream 7 clear transfer complete interrupt flag
#define REGISTER_DMA_HIFCR_CTCIF6         (uint32_t)0x007F0000UL        ///< DMA high interrupt flag clear register stream 6 clear transfer complete interrupt flag

#define REGISTER_DMA_LIFCR_CTCIF1         (uint32_t)0x00000800UL        ///< DMA low interrupt flag clear register stream 1 clear transfer complete interrupt flag
#define REGISTER_DMA_LIFCR_CTCIF2         (uint32_t)0x00100000UL        ///< DMA low interrupt flag clear register stream 2 clear transfer complete interrupt flag
#define REGISTER_DMA_LIFCR_CTCIF3         (uint32_t)0x0F000000UL        ///< DMA low interrupt flag clear register stream 3 clear transfer complete interrupt flag

#define REGISTER_FLASH_ACR_BASE_ADDRESS   (uint32_t)0x40023C00UL        ///< Flash access control register base address
#define REGISTER_FLASH_ACR_PRFTEN         (uint32_t)0x00000100UL        ///< Flash access control register prefetch enable
#define REGISTER_FLASH_ACR_ICEN           (uint32_t)0x00000200UL        ///< Flash access control register instruction cache enable
#define REGISTER_FLASH_ACR_DCEN           (uint32_t)0x00000400UL        ///< Flash access control register data cache enable
#define REGISTER_FLASH_ACR_ICRST          (uint32_t)0x00000800UL
#define REGISTER_FLASH_ACR_DCRST          (uint32_t)0x00001000UL
#define REGISTER_FLASH_ACR_LATENCY        (uint32_t)0x00000003UL        ///< Flash access control register latency

#define REGISTER_FLASH_KEYR_BASE_ADDRESS  (uint32_t)0x40023C04UL
#define REGISTER_FLASH_KEY1               (uint32_t)0x45670123UL
#define REGISTER_FLASH_KEY2               (uint32_t)0xCDEF89ABUL

#define REGISTER_FLASH_SR_BASE_ADDRESS    (uint32_t)0x40023C0CUL
#define REGISTER_FLASH_SR_RESET           (uint32_t)0x00000000UL
#define REGISTER_FLASH_SR_BSY_MSK         (uint32_t)0x00010000UL
#define REGISTER_FLASH_SR_BSY             (uint32_t)0x00010000UL
#define REGISTER_FLASH_SR_NOT_BSY         (uint32_t)0x00000000UL

#define REGISTER_FLASH_CR_BASE_ADDRESS    (uint32_t)0x40023C10UL
#define REGISTER_FLASH_CR_LOCK_MSK        (uint32_t)0x80000000UL
#define REGISTER_FLASH_CR_LOCKED          (uint32_t)0x80000000UL
#define REGISTER_FLASH_CR_NOT_LOCKED      (uint32_t)0x00000000UL
#define REGISTER_FLASH_CR_PG              (uint32_t)0x00000001UL
#define REGISTER_FLASH_CR_SER             (uint32_t)0x00000002UL
#define REGISTER_FLASH_CR_SNB             (uint32_t)0x00000078UL
#define REGISTER_FLASH_CR_STRT            (uint32_t)0x00010000UL
#define REGISTER_FLASH_CR_PSIZE           (uint32_t)0x00000300UL

/*Define TIMER1 Register */
#define REGISTER_TIMER1_BASE_ADDRESS      (uint32_t)0x40010400UL        ///< Timer1 base address
#define REGISTER_RCC_APB2ENR_TIMER8_EN    (uint32_t)0x00000002UL        ///< RCC AHB2 peripheral clock enable register timer8 enable

/*Base adresse of the NVIC*/
#define REGISTER_NVIC_BASE_ADDR           (uint32_t)0xE000E100UL        ///< NVIC base address

/*Base address syscfg*/
#define REGISTER_SYSCFG_EXTICR4_BASE_ADDRESS  (uint32_t)0x40013814UL        ///< SYSCFG external interrupt configuration register 4
#define REGISTER_SYSCFG_EXTICR2_BASE_ADDRESS  (uint32_t)0x4001380CUL        ///< SYSCFG external interrupt configuration register 2
#define REGISTER_SYSCFG_EXTICR1_BASE_ADDRESS  (uint32_t)0x40013808UL        ///< SYSCFG external interrupt configuration register 1
#define REGISTER_EXTI_IMR_BASE_ADDRESS        (uint32_t)0x40013C00UL        ///< EXTI IMR base address
#define REGISTER_EXTI_RTSR_BASE_ADDRESS       (uint32_t)0x40013C08UL        ///< EXTI RTSR base address
#define REGISTER_EXTI_PR_BASE_ADDRESS         (uint32_t)0x40013C14UL        ///< EXTI PR base address

/* VSYNC0 interrupt*/
#define REGISTER_PB15_EXTI_EN             (uint32_t)0x0001000UL         ///< PB15 EXTI enable
#define REGISTER_LINE15_EN                (uint32_t)0x0008000UL         ///< LINE15 enable
#define REGISTER_RTSR_LINE15_EN           (uint32_t)0x0008000UL         ///< RTSR LINE15 enable
#define REGISTER_PR_LINE15_EN             (uint32_t)0x0008000UL         ///< PR LINE15 enable
/* VSYNC1 interrupt*/
#define REGISTER_PC01_EXTI_EN             (uint32_t)0x0000020UL         ///< PC01 EXTI enable
#define REGISTER_LINE01_EN                (uint32_t)0x0000002UL         ///< LINE01 enable
#define REGISTER_RTSR_LINE01_EN           (uint32_t)0x0000002UL         ///< RTSR LINE01 enable
#define REGISTER_PR_LINE01_EN             (uint32_t)0x0000002UL         ///< PR LINE01 enable
/* Processing FPGA CRC interrupt*/
#define REGISTER_PC05_EXTI_EN             (uint32_t)0x0000020UL         ///< PC05 EXTI enable
#define REGISTER_LINE05_EN                (uint32_t)0x0000020UL         ///< LINE05 enable
#define REGISTER_RTSR_LINE05_EN           (uint32_t)0x0000020UL         ///< RTSR LINE05 enable
#define REGISTER_PR_LINE05_EN             (uint32_t)0x0000020UL         ///< PR LINE05 enable

/* Processing FPGA CRC interrupt*/
#define REGISTER_PC07_EXTI_EN             (uint32_t)0x0002000UL         ///< PC07 EXTI enable
#define REGISTER_LINE07_EN                (uint32_t)0x0000080UL         ///< LINE07 enable
#define REGISTER_RTSR_LINE07_EN           (uint32_t)0x0000080UL         ///< RTSR LINE07 enable
#define REGISTER_PR_LINE07_EN             (uint32_t)0x0000080UL         ///< PR LINE07 enable

#define REGISTER_GPIOH_BASE_ADDRESS   0x40021C00UL          ///< GPIO H base address
#define REGISTER_GPIOG_BASE_ADDRESS   0x40021800UL          ///< GPIO G base address
#define REGISTER_GPIOF_BASE_ADDRESS   0x40021400UL          ///< GPIO F base address
#define REGISTER_GPIOE_BASE_ADDRESS   0x40021000UL          ///< GPIO E base address
#define REGISTER_GPIOD_BASE_ADDRESS   0x40020C00UL          ///< GPIO D base address
#define REGISTER_GPIOC_BASE_ADDRESS   0x40020800UL          ///< GPIO C base address
#define REGISTER_GPIOB_BASE_ADDRESS   0x40020400UL          ///< GPIO B base address
#define REGISTER_GPIOA_BASE_ADDRESS   0x40020000UL          ///< GPIO A base address

//UART BASE Adresses
#define REGISTER_UART_BASE1_ADDRESS  (uint32_t)0x40011000UL       ///< Uart1 Base Address
#define REGISTER_UART_BASE2_ADDRESS  (uint32_t)0x40004400UL       ///< Uart2 Base Address
#define REGISTER_UART_BASE3_ADDRESS  (uint32_t)0x40004800UL       ///< Uart3 Base Address
#define REGISTER_UART_BASE4_ADDRESS  (uint32_t)0x40004C00UL       ///< Uart4 Base Address
#define REGISTER_UART_BASE5_ADDRESS  (uint32_t)0x40005000UL       ///< Uart5 Base Address
#define REGISTER_UART_BASE6_ADDRESS  (uint32_t)0x40011400UL       ///< Uart6 Base Address
#define REGISTER_UART_BASE7_ADDRESS  (uint32_t)0x40007800UL       ///< Uart7 Base Address
#define REGISTER_UART_BASE8_ADDRESS  (uint32_t)0x40007C00UL       ///< Uart8 Base Address
#define REGISTER_UART_BASE9_ADDRESS  (uint32_t)0x40011800UL       ///< Uart9 Base Address
#define REGISTER_UART_BASE10_ADDRESS (uint32_t)0x40011C00UL       ///< Uart10 Base Address

//Uart Data Register Base Receive Adresses
#define REGISTER_UART_DR_BASE1_ADDRESS  ((uint32_t)0x40011004UL)      ///< Uart1 Data Register Base Receive Address
#define REGISTER_UART_DR_BASE4_ADDRESS  ((uint32_t)0x40004C04UL)      ///< Uart4 Data Register Base Receive Address
#define REGISTER_UART_DR_BASE5_ADDRESS  ((uint32_t)0x40005004UL)      ///< Uart5 Data Register Base Receive Address
#define REGISTER_UART_DR_BASE6_ADDRESS  ((uint32_t)0x40011404UL)      ///< Uart6 Data Register Base Receive Address
#define REGISTER_UART_DR_BASE7_ADDRESS  ((uint32_t)0x40007804UL)      ///< Uart7 Data Register Base Receive Address
#define REGISTER_UART_DR_BASE10_ADDRESS ((uint32_t)0x40011C04UL)      ///< Uart10 Data Register Base Receive Address


//SPI Base Register
#define REGISTER_SPI5_BASE_ADDRESS     (uint32_t)(0x40015000U)     ///< SPI5 peripheral base address

#endif /* REGISTER_H */
