/**
 \file      error.h
 \author    mlanger, KAPPA optronics GmbH
 \date      11.04.2022
 \details   Error code for the different software errors

 \version   11.04.2022, mlanger
            - file created
 */

#ifndef ERROR_H
#define ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "types.h"

/*******************************************************
 ***         global defines and data types           ***
 *******************************************************/

/**
 \enum    T_ERROR
 \brief   Enum for different Errors
*/
typedef enum{
    NO_ERROR = 0U,              ///< Success (No error)
    ERROR_UART,                 ///< Uart Error
    ERROR_SPI,                  ///< Spi Error
    ERROR_I2C,                  ///< I2C Error
    ERROR_DMA,                  ///< DMA Error
    ERROR_ADC,                  ///< ADC Error
    ERROR_GPIO,                 ///< GPIO Error
    ERROR_EEPROM,               ///< EEPROM Error
    ERROR_FPGAINIT,             ///< FPGA Error
    ERROR_ALARM,                ///< Alarm Callback Error
    ERROR_SENSOR_AD7998,        ///< Sensor AD7998 Error
    ERROR_SENSOR_SHT3X,         ///< Sensor SHT3x Error
    ERROR_SENSOR_IMX392,        ///< Sensor IMX392 Error
    ERROR_PCA95XX,              ///< PCA95xx Error
    ERROR_LWIR   ,              ///< LWIR (DevALab) Error
    ERROR_EXTERNAL_INTERFACE,   ///< External Interface Error
    ERROR_CMC,                  ///< External Communication Error
    ERROR_DEFECTIVE_PIXEL,      ///< Defective Pixel Error
    ERROR_TESTPATTERN           ///< Test Pattern Error
}T_ERROR;

/*******************************************************
 ***               global variables                  ***
 *******************************************************/

/*******************************************************
 ***          global function declarations           ***
 *******************************************************/

/**
 \brief         Sets the specific error code.
 \param[in]     tError        the error code
*/
void error_SetError(T_ERROR tError);

/**
 \brief         Gets the error code.
 \return        Returns all the active errors.
*/
uint32_t error_GetError(void);

#ifdef __cplusplus
}
#endif

#endif /* ERROR_H */
