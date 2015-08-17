/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  typeDefine.h
* @brief Header file for basic types definitions
*
* Change Log:
*      Date                Who             What
*	   2015/01/09		  Qin Shi		  create
*******************************************************************************
*/

#pragma once

#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

#ifndef INOUT
#define INOUT
#endif

#ifndef ON
#define ON    1
#endif 

#ifndef OFF
#define OFF    0
#endif

#ifndef NULL
    #define NULL                 ((void*)0)
#endif

#ifndef AC_VAR_UNUSED
    #define AC_VAR_UNUSED(var)      (var = var)
#endif

/*Macro and type define*/
typedef         signed char             int8;
typedef         short                   int16;
typedef         int                     int32;

typedef         unsigned char           uint8;
typedef         unsigned short          uint16;
typedef         unsigned int            uint32;

/*!
 *  64 bit data types
 */

 typedef long long                      int64;
 typedef unsigned long long             uint64;



/* typedef for floating number representation using mantissa*2^exp */
typedef struct{
    int16  mant;   // signed Q1.15 mantissa
    int16     exp;    // the 2^exponent term 
}sfloat16;


typedef struct {
    uint16  mant;        // unsigned Q16 mantissa 
    int16     exp;         // the  2^exponent term 
}ufloat16;


#define AC_NUM_BYTES_IN_INT8               (sizeof(int8))

#define AC_NUM_BYTES_IN_INT16              (sizeof(int16))

#define AC_NUM_BYTES_IN_INT32              (sizeof(int32))

#define AC_NUM_BYTES_IN_INT40              (sizeof(int40))

#define AC_NUM_BYTES_IN_INT64              (sizeof(int64))

#define AC_NUM_BITS_IN_BYTE                8
   
