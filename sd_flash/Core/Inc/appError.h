/** 
 \file 		appError.h
 \author	sw, KAPPA opto-electronics GmbH
 \date 		Creation date: 24.01.2020
 \details	Error codes used in all application modules
			

 \version	24.01.2020 (sw)
			- created based on ACS-252			

 \note		The error codes are based on the common error codes in MC_LIB_common module kError.h.

 */

#ifndef APPERROR_H
#define APPERROR_H

#ifdef __cplusplus 
extern "C" {
#endif

#include "types.h"

//------------------------------------------------------------------------
//- global defines and data types
//------------------------------------------------------------------------

// Error codes
#define SUCCESS							0x00000000U     ///< success
#define GENERIC_ERROR					0x80000000U     ///< unspecified error
#define APPERROR_LENGTH_ERR				0xA0001200U     ///< data length error
#define APPERROR_PARAM_ERR				0xA0000700U     ///< parameter error
#define APPERROR_CRC_ERR				0xA0001000U     ///< EEPROM CRC failure
#define APPERROR_INIT_ERR				0xA0000E00U     ///< EEPROM initilization error
#define APPERROR_HW_ERR					0xA0000C00U     ///< hardware access error, all kDevice errors are mapped to this code
#define APPERROR_REG_ADDR_NA_ERR		0xA0000900U     ///< address not available
#define APPERROR_REG_ADDR_ACCESS_ERR	0xA0001400U     ///< address not available for this command type or access mode
#define APPERROR_CMD_NA_ERR				0xA0000800U     ///< command not available
#define APPERROR_DATA_TYPE_ERR			0xA0000600U     ///< data type invalid
#define APPERROR_TIMEOUT_ERR			0xA0000D00U     ///< timeout
#define APPERROR_BUSY_ERR				0xA0001A00U     ///< device busy, command not executed
	
//------------------------------------------------------------------------
//- global variables
//------------------------------------------------------------------------


//------------------------------------------------------------------------
//- global functions
//------------------------------------------------------------------------

#ifdef __cplusplus
}
#endif

#endif /* APPERROR_H */
