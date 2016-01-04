/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  VisualizeControl.h
* @brief function to control 3d engine, 
*         include zoom in/out,move,mouse and keyboard response function
*
* Change Log:
*      Date                Who             What
*      2015/02/05         Xin Shao        Create
*******************************************************************************
*/
#pragma once

#include "Visualization.h"

typedef enum
{
	VideoPlayEnum_normal = 0,
	VideoPlayEnum_1_2,
	VideoPlayEnum_pause
}VideoPlayEnum;

extern OPENGL_3D_ENGINE *engine3DPtr;
extern eyeLookAt_t serverEyeInfo[];
extern VideoPlayEnum videoPlaySpeed;

#define DEFAULT_WINDOW_HEIGHT 400
#define DEFAULT_WINDOW_WIDTH  600
void Init3DEngine(void);
void delete3DEngine(void);
void mouseButtonFunc(int button, int state, int x, int y);
void mouseMoveFunc(int x, int y);
void specialKeyFunc(int key, int x, int y);
void keyboardFunc(unsigned char key, int x, int y);
void reshapeWindow(int width, int height);
void myDisplay(void);
void myTimer(int value);
