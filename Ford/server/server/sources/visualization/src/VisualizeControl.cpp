/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  VisualizeControl.cpp
* @brief function to control 3d engine, 
*         include zoom in/out,move,mouse and keyboard response function
*
* Change Log:
*      Date                Who             What
*      2015/02/06         Xin Shao        Create
*******************************************************************************
*/
#include <stdio.h>
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include <gl/glut.h>
#include <math.h>

#include "Visualization.h"
#include "messageProcessClass.h"
#include "appInitCommon.h"
#include "VisualizeControl.h"
#include "configure.h"

OPENGL_3D_ENGINE *engine3DPtr;
eyeLookAt_t serverEyeInfo[2];

int down_x;
int down_y;
int down_button = -1;
int window_height = DEFAULT_WINDOW_HEIGHT;
int window_width = DEFAULT_WINDOW_WIDTH;

void load_all_textures(void)
{
#if((RD_LOCATION&RD_NATION_MASK) == RD_GERMAN)
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/10100.bmp",1);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/12300.bmp",2);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/13100.bmp",3);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/13310.bmp",4);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/13810.bmp",5);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/20500.bmp",6);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/20600.bmp",7);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/20930.bmp",8);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/21500.bmp",9);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/22220.bmp",10);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/22400.bmp",11);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/23700.bmp",12);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/23900.bmp",13);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/24000.bmp",14);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/25000.bmp",15);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/25900.bmp",16);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/26100.bmp",17);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/26210.bmp",18);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/27452.bmp",19);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/27453.bmp",20);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/27454.bmp",21);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/27455.bmp",22);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/27456.bmp",23);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/27458.bmp",24);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/28300.bmp",25);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/28600.bmp",26);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/30100.bmp",27);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/30600.bmp",28);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/31400.bmp",29);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/33100.bmp",30);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/33600.bmp",31);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/35010.bmp",32);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/99900.bmp",33);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/20910.bmp",34);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/25400.bmp",35);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/26700.bmp",36);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/31401.bmp",37);
	engine3DPtr->load_bmp24_texture("./resource/Germany/bmp/00000.bmp",38);

#else if((RD_LOCATION&RD_NATION_MASK) == RD_UNIT_STATES)

	engine3DPtr->load_bmp24_texture("./resource/US/bmp/1.bmp",1);
    engine3DPtr->load_bmp24_texture("./resource/US/bmp/2.bmp",2);
    engine3DPtr->load_bmp24_texture("./resource/US/bmp/3.bmp",3);
    engine3DPtr->load_bmp24_texture("./resource/US/bmp/4.bmp",4);
    engine3DPtr->load_bmp24_texture("./resource/US/bmp/5.bmp",5);
    engine3DPtr->load_bmp24_texture("./resource/US/bmp/6.bmp",6);
    engine3DPtr->load_bmp24_texture("./resource/US/bmp/7.bmp",7);
    engine3DPtr->load_bmp24_texture("./resource/US/bmp/8.bmp",8);
#endif
}

//Initialize the 3D engine, load all bitmap files
void Init3DEngine(void)
{
	engine3DPtr = new OPENGL_3D_ENGINE();
	engine3DPtr->setMode(ServerMode_3DEngine);
	load_all_textures();
}

//delete 
void delete3DEngine(void)
{
	delete engine3DPtr;
}

//
void updateServerEye(eyeLookAt_t * lookAt,GLfloat angle, GLfloat range)
{
	lookAt->eyePosition.x -= range*cos(PI/180*angle);
	lookAt->eyePosition.z -= range*sin(PI/180*angle);
	lookAt->lookatPosition.x -= range*cos(PI/180*angle);
	lookAt->lookatPosition.z -= range*sin(PI/180*angle);
}

//mouse button response function
void mouseButtonFunc(int button, int state, int x, int y)
{
	if(GLUT_DOWN == state)
	{
		if((GLUT_RIGHT_BUTTON == button )||(GLUT_MIDDLE_BUTTON == button))
		{
			down_x = x;
			down_y = y;
		}
		down_button = button;
	}
}

//mouse move response function
void mouseMoveFunc(int x, int y)
{
	//move
	if(GLUT_RIGHT_BUTTON == down_button)
	{
		if((x != down_x) || (y != down_y))
		{
			GLfloat x_dif = ((GLfloat)(x - down_x))*1680/window_width;
			GLfloat y_dif = ((GLfloat)(y - down_y))*1050/window_height;
			GLfloat screenAngle = -180*atan2(x_dif,y_dif)/PI;
			GLfloat screenDis   = -sqrt(x_dif*x_dif+y_dif*y_dif);
			GLfloat serverAngle = engine3DPtr->getServerAngle();
			updateServerEye(&serverEyeInfo[0],(serverAngle+screenAngle),screenDis/320*serverEyeInfo[0].eyePosition.y);
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
			down_x = x;
			down_y = y;
		}
	}

	//zoomIn and zoomOut
	if(GLUT_MIDDLE_BUTTON == down_button)
	{
		GLfloat y_dif = (y - down_y)/2;
	    
		if(y_dif > 0)//zoom in
		{
		    serverEyeInfo[0].eyePosition.y -= y_dif;
			serverEyeInfo[0].eyePosition.y = (serverEyeInfo[0].eyePosition.y < 1.0f)?1.0f:serverEyeInfo[0].eyePosition.y;
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
			down_x = x;
			down_y = y;
		}

		if(y_dif < 0)//zoom out
		{
		    serverEyeInfo[0].eyePosition.y -= y_dif;
			serverEyeInfo[0].eyePosition.y = (serverEyeInfo[0].eyePosition.y < 1.0f)?1.0f:serverEyeInfo[0].eyePosition.y;
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
			down_x = x;
			down_y = y;
		}
	}

}

//special key board response function
void specialKeyFunc(int key, int x, int y)
{
	switch(key)
	{
	case GLUT_KEY_DOWN:
		{
			GLfloat serverAngle = engine3DPtr->getServerAngle();
			updateServerEye(&serverEyeInfo[0],(serverAngle+180),4);
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
		}
		break;
	case GLUT_KEY_UP:
		{
			GLfloat serverAngle = engine3DPtr->getServerAngle();
			updateServerEye(&serverEyeInfo[0],(serverAngle),4);
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
		}
		break;
	case GLUT_KEY_LEFT:
		{
			GLfloat serverAngle = engine3DPtr->getServerAngle();
			updateServerEye(&serverEyeInfo[0],(serverAngle+270),4);
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
		}
		break;
	case GLUT_KEY_RIGHT:
		{
			GLfloat serverAngle = engine3DPtr->getServerAngle();
			updateServerEye(&serverEyeInfo[0],(serverAngle+90),4);
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
		}
		break;
	case GLUT_KEY_PAGE_UP:
		{
			GLfloat serverAngle = engine3DPtr->getServerAngle();
			serverAngle += 2.0f;
			engine3DPtr->setServerAngle(serverAngle);
		}
		break;
	case GLUT_KEY_PAGE_DOWN:
		{
			GLfloat serverAngle = engine3DPtr->getServerAngle();
			serverAngle -= 2.0f;
			engine3DPtr->setServerAngle(serverAngle);
		}
		break;
	default:
		break;
	}
}

//key board response function
void keyboardFunc(unsigned char key, int x, int y)
{
	switch(key)
	{
	case 127:
	// reset database key event
		{
			messageProcessClass statusMessage;
			diffRptMsg_t* statusRptMsgPtr = statusMessage.getDiffRptMsg();
			int headerLen = sizeof(statusRptMsgPtr->msgHeader) + sizeof(statusRptMsgPtr->payloadHeader.pduHeader[0]);
			statusMessage.setMsgHeader((uint32*)statusRptMsgPtr,headerLen,STATUS_UPDATE_RPT_MSG,0,highLevel_e,4,1);
			statusRptMsgPtr->payloadHeader.tlvArray[0].tag = 1;
			statusRptMsgPtr->payloadHeader.tlvArray[0].len = 2;
			statusRptMsgPtr->payloadHeader.tlvArray[0].value = 3;
			messageQueue_gp->push(&statusMessage);
			ReleaseSemaphore(g_readySema_msgQueue,1,NULL);
		}
		break;
	case 73: //zoom in
	case 105:
		{
			serverEyeInfo[0].eyePosition.y -= 3;
			serverEyeInfo[0].eyePosition.y = (serverEyeInfo[0].eyePosition.y < 1.0f)?1.0f:serverEyeInfo[0].eyePosition.y;
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
		}
		break;
	case 79: //zoom out
	case 111:
		{
			serverEyeInfo[0].eyePosition.y += 3;
			serverEyeInfo[0].eyePosition.y = (serverEyeInfo[0].eyePosition.y < 1.0f)?1.0f:serverEyeInfo[0].eyePosition.y;
			engine3DPtr->setServerEyeLookat(1,serverEyeInfo);
			engine3DPtr->SwapServerEyeBuffer();
		}
		break;
	case 83: //turn on/off traffic sign
	case 115:
		engine3DPtr->setSignFlag();
		break;
	default:
		break;
	}

}

void reshapeWindow(int width, int height)
{
	engine3DPtr->setWindow(width, height);
	window_height = height;
	window_width = width;
}

void myDisplay(void)
{
	engine3DPtr->DrawAll();
}

void myTimer(int value)
{
	if(1 == value)
	{
		glutTimerFunc((unsigned int)(1000/FRAME_NUM_PER_SECOND),&myTimer,1);
		myDisplay();
	}
}

