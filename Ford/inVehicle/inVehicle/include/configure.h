/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  configure.h
* @brief Header file for configure
*
* Change Log:
*      Date                Who             What
*	   2015/08/07		  Xin Shao		  create
*******************************************************************************
*/
#pragma once
#include "typeDefine.h"

#define RD_GERMAN_MUNICH_AIRPORT  (0x1001)
#define RD_GERMAN_LEHRE           (0x1002)
#define RD_US_DETROIT             (0x2001)
#define RD_US_PALO_ALTO           (0x2002)

#define RD_NATION_MASK            (0xf000)
#define RD_GERMAN                 (0x1000)
#define RD_UNIT_STATES            (0x2000)

#define RD_SIGN_DETECT_OFF            0
#define RD_SIGN_DETECT_WHITE_BLACK    1
#define RD_SIGN_DETECT_COLOR          2

#if defined(_FRANKFORT_CAMERA)
#define RD_USE_CAMERA          ON
#define RD_IMAGE_BUFFER_FILE   ON
#define RD_LOCATION            RD_GERMAN_MUNICH_AIRPORT
#define RD_SIGN_DETECT         RD_SIGN_DETECT_COLOR
#define RD_ROAD_DETECT         ON

#elif defined(_FRANKFORT_VIDEO)
#define RD_USE_CAMERA          OFF
#define RD_IMAGE_BUFFER_FILE   ON
#define RD_LOCATION            RD_GERMAN_MUNICH_AIRPORT
#define RD_SIGN_DETECT         RD_SIGN_DETECT_COLOR
#define RD_ROAD_DETECT         ON
#elif defined(_DETROIT_VIDEO)
#define RD_USE_CAMERA          OFF
#define RD_IMAGE_BUFFER_FILE   ON
#define RD_LOCATION            RD_US_DETROIT
#define RD_SIGN_DETECT         RD_SIGN_DETECT_WHITE_BLACK
#define RD_ROAD_DETECT         ON
#elif defined(_DE_LEHRE_VIDEO)
#define RD_USE_CAMERA          OFF
#define RD_IMAGE_BUFFER_FILE   ON
#define RD_LOCATION            RD_GERMAN_LEHRE
#define RD_SIGN_DETECT         RD_SIGN_DETECT_OFF
#define RD_ROAD_DETECT         ON
#else
#define RD_USE_CAMERA          OFF
#define RD_IMAGE_BUFFER_FILE   ON
#define RD_LOCATION            RD_GERMAN_MUNICH_AIRPORT
#define RD_SIGN_DETECT         RD_SIGN_DETECT_COLOR
#define RD_ROAD_DETECT         ON
#endif


  



