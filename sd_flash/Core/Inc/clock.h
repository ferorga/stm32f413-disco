#ifndef __CLOCK_H__
#define __CLOCK_H__

#include "types.h"

/*******************************************************
 ***         global defines and data types           ***
 *******************************************************/
 
///Enum to select RCC Register
typedef enum
{
    RCC_AHB1,    //< RCC_AHB1ENR peripheral clock enable register
    RCC_AHB3,    //< RCC_AHB3EMR peripheral clock enable register
    RCC_APB1,    //< RCC_APB1 peripheral clock enable register
    RCC_APB2     //< RCC_APB2 peripheral clock enable register
}T_CLOCK_RCC;

/*******************************************************
 ***          global function declarations           ***
 *******************************************************/

/**
 \brief     Function to init the system configuration for the clocks
*/
void clock_Init(void);

void clock_DeInit(void);

/**
 \brief Function to set the CLOCK_RCC Registers
 \param[in]     tRegSelect      RCC Register
 \param[in]     ulValue         Value which is to be set
*/
void clock_SetRCCRegister(T_CLOCK_RCC tRegSelect, uint32_t ulValue);

/**
 \brief Function to get the CLOCK_RCC Registers
 \param[in]     tRegSelect      RCC Register
 \return        Pointer to the RCC Register
*/
volatile T_HWREG* clock_GetRCCRegister(T_CLOCK_RCC tRegSelect);

#ifdef __cplusplus
}
#endif

#endif /* __CLOCK_H__ */
