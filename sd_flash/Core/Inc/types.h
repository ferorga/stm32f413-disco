/**
 \file      types.h
 \author    mko, KAPPA optronics GmbH
 \date      13.04.2022
 \details   data types used in the whole project

 \version   13.04.2022, mko
          - file created
*/
/*PRQA S 0631 EOF # Suppress!*/
#ifndef TYPES_H
#define TYPES_H

/*******************************************************
 ***          provided  define constant              ***
 *******************************************************/
#define ON           1U                 ///< on value
#define OFF          0U                 ///< off value
#define FALSE       ((bool_t)(0==1))    ///< boolean false
#define TRUE        ((bool_t)(1==1))    ///< boolean true
#define NULL_PTR    (void*)0U           ///< null pointer

#define MAX_UINT32_VALUE    0xFFFFFFFFU ///< maximum uint32 value

/*******************************************************
 ***          provided  type                         ***
 *******************************************************/
/**
 \brief    types definition
*/
#ifndef __cplusplus
typedef _Bool              bool_t;    ///< boolean type
#else
typedef bool               bool_t;    ///< boolean type
#endif
typedef unsigned char      uint8_t;   ///< unsigned byte type (8bit)
typedef signed char        int8_t;    ///< signed byte type (8bit)
typedef unsigned short     uint16_t;  ///< unsigned word type (16bit)
typedef signed short       int16_t;   ///< signed word type (16bit)
typedef unsigned int       uint32_t;  ///< unsigned integer type (32bit)
typedef signed int         int32_t;   ///< signed integer type (32bit)
typedef unsigned long long uint64_t;  ///< unsigned long type (64bit)
typedef signed long long   int64_t;   ///< usigned long type (64bit)
typedef float              float32_t; ///< floating point type (32bit)
typedef double             double32_t;///< double type (32bit)

typedef void (*T_FUNC_POINTER)(void); ///< function pointer type

/**
 \union    T_HWREG
 \brief    Type for general Register access
*/
typedef union
{
 uint32_t  ulHWREG;      ///< Hardware register 32 bit
 uint16_t  wHWREG[2];    ///< Hardware register sperated in 2x 16 bit word
 uint8_t   ucHWREG[4];   ///< Hardware register sperated in 4x 8 bit byte
}T_HWREG;

#include "error.h"

#endif /* TYPES_H */
