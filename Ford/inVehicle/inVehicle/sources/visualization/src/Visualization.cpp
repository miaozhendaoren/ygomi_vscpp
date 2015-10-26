/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Visualization.cpp
* @brief A sample 3D engine, provide some function to add road, sign, and perspective information.
*        road and sign information can load in once or each seconds.
*        perspective information is based one road and sign coordinates, each frame has its own information.
*
* Change Log:
*      Date                Who             What
*      2014/12/23         Xin Shao        Create
*******************************************************************************
*/

//only use function in openGL 1.1
#include <stdio.h>
#include <stdlib.h>
#include <gl/glut.h>
#include <string.h>
#include <windows.h>
#include <math.h>

#include "Visualization.h"

#define ColoredVertex(c, v) { glColor3fv(c); glVertex3fv(v);}

OPENGL_3D_ENGINE::OPENGL_3D_ENGINE()
{
	_winWidth = 600;
	_winHeight = 400;
	_signFlag = true;
	serverHeadAngle = 0;
	lineBackBufIdx = 0;
	roadLineBackBufIdx = 0;
	signBackbufIdx = 0;
	eyeBackBufIdx = 0;
	charBackBufIdx = 0;
	serverEyeBackBufIdx = 0;
	eyeLookaheadBackBufIdx = 0;
	quadBackBufIdx = 0;
	serverCharBackBufIdx = 0;
	CarShowList = 0;

	//signBuffer[0].number = 0;
	//lineBuffer[0].number = 0;
	//roadLineBuffer[0].number = 0;
	charBuffer[0].number = 0;
	eyeBuffer[0].number  = 0;
	eyeBuffer[0].index  = 0;
	serverEyeBuffer[0].number = 0;
	serverEyeBuffer[0].index = 0;
	eyeBufferLookahead[0].number = 0;
	eyeBufferLookahead[0].index = 0;
	//quadBuffer[0].number = 0;

	//signBuffer[1].number = 0;
	//roadLineBuffer[1].number = 0;
	//lineBuffer[1].number = 0;
	charBuffer[1].number = 0;
	eyeBuffer[1].number  = 0;
	eyeBuffer[1].index  = 0;
	serverEyeBuffer[1].number = 0;
	serverEyeBuffer[1].index = 0;
	eyeBufferLookahead[1].number = 0;
	eyeBufferLookahead[1].index = 0;
	//quadBuffer[1].number = 0;

	glEnable(GL_TEXTURE_2D);
	glEnable(GL_DEPTH_TEST);


	CreateSignRoadSideShowList();
	CreateSignRoadOnShowList();
	CreateCharaterShowList();

	memset(texturelist,0,sizeof(GLuint)*MAX_BUFFER_DEPTH_2D_TXETURE);

	hMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(hMutex);
	drawMode = ClientMode_3DEngine;
}

OPENGL_3D_ENGINE::~OPENGL_3D_ENGINE()
{
	int i = 0;
	//delete the show list
	glDeleteLists(SignShowList_roadSide,1);
	glDeleteLists(CharShowList,128);

	//delete the TEXTURE_2D
	for(i = 0; i < MAX_BUFFER_DEPTH_2D_TXETURE; i++)
	{
		if(texturelist[i] != 0)
		{
			glDeleteTextures(1,&texturelist[i]);
		}
	}

	//delete the Mutex
	CloseHandle(hMutex);
}

//copy the Sign information 
GLboolean OPENGL_3D_ENGINE::AddSignInfo(
	vector<signInfo_t>& buffer)
{
	for(int index = 0; index < buffer.size(); index++)
	{
		signBuffer[signBackbufIdx].push_back(buffer[index]);
	}
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::AddQuadInfo(int number, quadInfo_t* buffer)
{
	for(int index = 0; index < number; index++)
	{
		quadBuffer[quadBackBufIdx].push_back(buffer[index]);
	}

	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::AddOneLineInfo(lineTypeEnum_t type, baseColor_t color, vector<point3DFloat_t>& buffer)
{
	lineInfo_t tempLine;
	tempLine.color = color;
	tempLine.type = type;
	for(int index = 0; index < buffer.size(); index++)
	{
		tempLine.position.push_back(buffer[index]);
	}

	lineBuffer[lineBackBufIdx].push_back(tempLine);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::AddOneRoadLineInfo(lineTypeEnum_t type, baseColor_t color, vector<point3DFloat_t>& buffer)
{
	lineInfo_t tempLine;
	tempLine.color = color;
	tempLine.type = type;
	for(int index = 0; index < buffer.size(); index++)
	{
		tempLine.position.push_back(buffer[index]);
	}

	roadLineBuffer[roadLineBackBufIdx].push_back(tempLine);

	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::AddOneServerCharInfo(drawServerCharInfo_t &charInfo)
{
	serverCharBuffer[serverCharBackBufIdx].push_back(charInfo);
	return true;
}

GLboolean OPENGL_3D_ENGINE::setLookAheadEyeLookat(int number, 
	eyeLookAt_t* buffer)
{
	//check if the sign info is larger then internal buffer
	if((eyeBufferLookahead[eyeLookaheadBackBufIdx].number+number) > MAX_BUFFER_DEPTH_3D_ENGINE)
	{
		return GL_FALSE;
	}

	//copy the content of signInfo_t information
	memcpy((void*)&eyeBufferLookahead[eyeLookaheadBackBufIdx].buffer[eyeBufferLookahead[eyeLookaheadBackBufIdx].number], (void*)buffer,number*sizeof(eyeLookAt_t));

	//update the number
	eyeBufferLookahead[eyeLookaheadBackBufIdx].number += number;

	return GL_TRUE;
}
//
GLboolean OPENGL_3D_ENGINE::setEyeLookat(int number,
	eyeLookAt_t* buffer)
{
	//check if the sign info is larger then internal buffer
	if((eyeBuffer[eyeBackBufIdx].number+number) > MAX_BUFFER_DEPTH_3D_ENGINE)
	{
		return GL_FALSE;
	}

	//copy the content of signInfo_t information
	memcpy((void*)&eyeBuffer[eyeBackBufIdx].buffer[eyeBuffer[eyeBackBufIdx].number], (void*)buffer,number*sizeof(eyeLookAt_t));

	//update the number
	eyeBuffer[eyeBackBufIdx].number += number;

	return GL_TRUE;

}         

GLboolean OPENGL_3D_ENGINE::setServerEyeLookat(int number, eyeLookAt_t* buffer)
{
	//check if the sign info is larger then internal buffer
	if((serverEyeBuffer[serverEyeBackBufIdx].number+number) > MAX_BUFFER_DEPTH_3D_ENGINE)
	{
		return GL_FALSE;
	}

	//copy the content of signInfo_t information
	memcpy((void*)&serverEyeBuffer[serverEyeBackBufIdx].buffer[serverEyeBuffer[serverEyeBackBufIdx].number], (void*)buffer,number*sizeof(eyeLookAt_t));

	//update the number
	serverEyeBuffer[serverEyeBackBufIdx].number += number;

	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::setDrawChar(drawCharInfo_t *buffer)
{
	if(charBuffer[charBackBufIdx].number >= MAX_BUFFER_DEPTH_DRAW_CHAR)
	{
		return GL_FALSE;
	}

	memcpy((void*)&charBuffer[charBackBufIdx].buffer[charBuffer[charBackBufIdx].number], buffer,sizeof(drawCharInfo_t));
	charBuffer[charBackBufIdx].number += 1;

	return GL_TRUE;
}
//change the backend buffer and front buffer(draw buffer)         
GLboolean OPENGL_3D_ENGINE::Swap3DBuffers(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
	charBackBufIdx ^= 1;
	lineBackBufIdx ^= 1;
	roadLineBackBufIdx ^= 1;
	signBackbufIdx ^= 1;
	quadBackBufIdx ^= 1;
	//signBuffer[signBackbufIdx].number = 0;
	charBuffer[charBackBufIdx].number = 0;
	//lineBuffer[lineBackBufIdx].number = 0;
	//roadLineBuffer[roadLineBackBufIdx].number = 0;
	//quadBuffer[quadBackBufIdx].number = 0;
	signBuffer[signBackbufIdx].clear();
	lineBuffer[lineBackBufIdx].clear();
	roadLineBuffer[roadLineBackBufIdx].clear();
	quadBuffer[quadBackBufIdx].clear();
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::SwapQuadBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE);
	quadBackBufIdx ^= 1;
	//quadBuffer[quadBackBufIdx].number = 0;
	quadBuffer[quadBackBufIdx].clear();
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

//change the backend Sign information(road furniture) buffer
GLboolean OPENGL_3D_ENGINE::SwapSignBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
	signBackbufIdx ^= 1;
	//signBuffer[signBackbufIdx].number = 0;
	signBuffer[signBackbufIdx].clear();
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

//change the backend line buffer
GLboolean OPENGL_3D_ENGINE::SwapLineBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
	lineBackBufIdx ^= 1;
	//lineBuffer[lineBackBufIdx].number = 0;
	lineBuffer[lineBackBufIdx].clear();
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::SwapRoadLineBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
	roadLineBackBufIdx ^= 1;
	//roadLineBuffer[roadLineBackBufIdx].number = 0;
	roadLineBuffer[roadLineBackBufIdx].clear();
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::SwapEyeBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
	eyeBackBufIdx ^= 1;
	eyeBuffer[eyeBackBufIdx].number = 0;
	eyeBuffer[eyeBackBufIdx].index = 0;
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::SwapLookaheadEyeBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
	eyeLookaheadBackBufIdx ^= 1;
	eyeBufferLookahead[eyeLookaheadBackBufIdx].number = 0;
	eyeBufferLookahead[eyeLookaheadBackBufIdx].index = 0;
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::SwapServerEyeBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
	serverEyeBackBufIdx ^= 1;
	serverEyeBuffer[serverEyeBackBufIdx].number = 0;
	serverEyeBuffer[serverEyeBackBufIdx].index = 0;
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::SwapCharBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE);
	charBackBufIdx ^= 1;
	charBuffer[charBackBufIdx].number = 0;
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

GLboolean OPENGL_3D_ENGINE::SwapServerCharBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE);
	serverCharBackBufIdx ^= 1;
	serverCharBuffer[serverCharBackBufIdx].clear();
	ReleaseMutex(hMutex);
	return GL_TRUE;
}

void OPENGL_3D_ENGINE::setMode(mode3DEngineEnum_t mode)
{
	WaitForSingleObject(hMutex,INFINITE);
	drawMode = mode;
	ReleaseMutex(hMutex);
}

mode3DEngineEnum_t OPENGL_3D_ENGINE::getMode(void)
{
	return drawMode;
}

void OPENGL_3D_ENGINE::setServerAngle(GLfloat angle)
{
	WaitForSingleObject(hMutex,INFINITE);
	serverHeadAngle = angle;
	ReleaseMutex(hMutex);
}
	
GLfloat OPENGL_3D_ENGINE::getServerAngle(void)
{
	return serverHeadAngle;
}

void OPENGL_3D_ENGINE::setClientEyeOnce(eyeLookAt_t eye, modeViewEnum_t mode,GLfloat aspect)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(90,aspect,1,2000);
	
	
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	GLfloat angle = ComputeAngle(&eye.eyePosition, &eye.lookatPosition);

	gluLookAt(eye.eyePosition.x-Driver_VIEW_OFFSET*sin(-angle*PI/180),eye.eyePosition.y,eye.eyePosition.z+Driver_VIEW_OFFSET*cos(-angle*PI/180),
		      eye.lookatPosition.x-Driver_VIEW_OFFSET*sin(-angle*PI/180),eye.lookatPosition.y,eye.lookatPosition.z+Driver_VIEW_OFFSET*cos(-angle*PI/180), 
			  0, 1, 0);

	//draw blue sky
	glPushMatrix();
	glTranslatef(eye.eyePosition.x, 0, eye.eyePosition.z);
	DrawBlueSky();
	glPopMatrix();

	if(ViewEnum_Driver == mode)
	{
#if  0  //green car
		glPushMatrix();
		GLfloat angle = ComputeAngle(&eye.eyePosition, &eye.lookatPosition);
		glTranslatef(eye.eyePosition.x + 0.3*cos(-angle*PI/180)-1.5*sin(-angle*PI/180), 1.5, eye.eyePosition.z + 0.3*sin(-angle*PI/180)+1.5*cos(-angle*PI/180));

		glRotatef(angle-90, 0,1,0);
		glScalef(0.2,0.2,0.2);
		//glScalef(0.5,0.5,0.5);
		if(0 != CarShowList)
		{
			glCallList(CarShowList);
		}
		glPopMatrix();
#endif

#if 0  //red car
		glPushMatrix();
		//GLfloat angle = ComputeAngle(&eye.eyePosition, &eye.lookatPosition);
		
		glTranslatef(eye.eyePosition.x + 1.2*cos(-angle*PI/180)-CAR_POS_OFFSET*sin(-angle*PI/180), 1.5, eye.eyePosition.z + 1.2*sin(-angle*PI/180)+CAR_POS_OFFSET*cos(-angle*PI/180));

		glRotatef(angle-90, 0,1,0);
		
		glScalef(0.0002,0.0002,0.0002);
		if(0 != CarShowList)
		{
			glCallList(CarShowList);
		}
		glPopMatrix();
#endif

	}

}

void OPENGL_3D_ENGINE::setServerEyeOnce(eyeLookAt_t eye, GLfloat aspect)
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(120,aspect,1,2000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(eye.eyePosition.x,eye.eyePosition.y,eye.eyePosition.z,eye.lookatPosition.x,eye.lookatPosition.y,eye.lookatPosition.z, cos(PI/180*serverHeadAngle), 0, sin(PI/180*serverHeadAngle));

}

void OPENGL_3D_ENGINE::DrawFrontBufferClient()
{
	int signIdx,lineIdx,quadIdx;
	int signFrontBufIdx = signBackbufIdx^1;
	int lineFrontBufIdx = lineBackBufIdx^1;
	int roadLineFrontBufIdx = roadLineBackBufIdx^1;
	int quadFrontBufIdx = quadBackBufIdx^1;

	if(_signFlag)
	{
		//draw all the sign
		for(signIdx = 0; signIdx < signBuffer[signFrontBufIdx].size(); signIdx++)
		{
			if(4 == signBuffer[signFrontBufIdx][signIdx].sideFlag)
			{
				DrawSignOnRoad(signBuffer[signFrontBufIdx][signIdx]);
			}else
			{
				DrawSignClient(signBuffer[signFrontBufIdx][signIdx]);
			}
			
		}
	}

	//draw road
	list<lineInfo_t>::iterator lineIter = roadLineBuffer[roadLineFrontBufIdx].begin();

	for(lineIdx = 0; lineIdx < (roadLineBuffer[roadLineFrontBufIdx].size()/2); lineIdx++)
	{
		DrawRoadwithLine(&(*lineIter++),&(*lineIter++));//,&roadLineBuffer[roadLineFrontBufIdx].line[lineIdx*2+1]);
		//DrawRoadSide(&roadLineBuffer[roadLineFrontBufIdx].line[lineIdx*2]);
		//DrawRoadSide(&roadLineBuffer[roadLineFrontBufIdx].line[lineIdx*2+1]);
	}

	//draw all the vectors
	list<lineInfo_t>::iterator lineIter2 = lineBuffer[lineFrontBufIdx].begin();
	while(lineIter2 != lineBuffer[lineFrontBufIdx].end())
	{
		DrawLine(&(*lineIter2),ClientMode_3DEngine);
		lineIter2++;
	}

	//draw quad sharp
	for(quadIdx = 0; quadIdx < quadBuffer[quadFrontBufIdx].size(); quadIdx++)
	{
		DrawQuad(quadBuffer[quadFrontBufIdx][quadIdx]);
	}
}

void OPENGL_3D_ENGINE::DrawFrontBufferServer()
{
	int signIdx,lineIdx,quadIdx, serverCharIdx;
	int charFrontBufIdx = charBackBufIdx^1;
	int signFrontBufIdx = signBackbufIdx^1;
	int lineFrontBufIdx = lineBackBufIdx^1;
	int roadLineFrontBufIdx = roadLineBackBufIdx^1;
	int quadFrontBufIdx = quadBackBufIdx^1;
	int serverCharFrontBufIdx = serverCharBackBufIdx^1;

	if(_signFlag)
	{
		//draw all the sign
		for(signIdx = 0; signIdx < signBuffer[signFrontBufIdx].size(); signIdx++)
		{
			if(4 == signBuffer[signFrontBufIdx][signIdx].sideFlag)
			{
				DrawSignOnRoad(signBuffer[signFrontBufIdx][signIdx]);
			}else
			{
				DrawSignServer(signBuffer[signFrontBufIdx][signIdx]);
			}
		}
	}

		//draw road
	list<lineInfo_t>::iterator lineIter = roadLineBuffer[roadLineFrontBufIdx].begin();

	for(lineIdx = 0; lineIdx < (roadLineBuffer[roadLineFrontBufIdx].size()/2); lineIdx++)
	{
		DrawRoadwithLine(&(*lineIter++),&(*lineIter++));//,&roadLineBuffer[roadLineFrontBufIdx].line[lineIdx*2+1]);
		//DrawRoadSide(&roadLineBuffer[roadLineFrontBufIdx].line[lineIdx*2]);
		//DrawRoadSide(&roadLineBuffer[roadLineFrontBufIdx].line[lineIdx*2+1]);
	}

	//draw all the vectors
	//list<lineInfo_t>::iterator lineIter2 = lineBuffer[lineFrontBufIdx].begin();
	//while(lineIter2 != lineBuffer[lineFrontBufIdx].end())
	//{
	//	DrawLine(&(*lineIter2),ServerMode_3DEngine);
	//	lineIter2++;
	//}

	//draw quad sharp
	for(quadIdx = 0; quadIdx < quadBuffer[quadFrontBufIdx].size(); quadIdx++)
	{
		DrawQuad(quadBuffer[quadFrontBufIdx][quadIdx]);
	}
	
	//draw server side char
	baseColor_t color;
	color.R = 1;
	color.G = 0;
	color.B = 0;
	//selectFont(8, ANSI_CHARSET);
	//glScalef(0.5,0.5,0.5);
	for(serverCharIdx = 0; serverCharIdx < serverCharBuffer[serverCharFrontBufIdx].size(); serverCharIdx++)
	{
		DrawChar(serverCharBuffer[serverCharFrontBufIdx][serverCharIdx].position, serverCharBuffer[serverCharFrontBufIdx][serverCharIdx].drawChar, color);
	}
}

void OPENGL_3D_ENGINE::DrawSpliteLine()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(90,1,1,100);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0, 1, 0, 0, 0, 0, 1, 0, 0);

	glPushMatrix();
	glColor3f(0.25f,0.25f,0.25f);

	glBegin(GL_QUADS);
	glVertex3f(2, 0, -2);
	glVertex3f(-2, 0, -2);
	glVertex3f(-2, 0, 2);
	glVertex3f(2, 0, 2);
	glEnd();
	glPopMatrix();
}

void OPENGL_3D_ENGINE::DrawCharView(char *drawChar,GLfloat red, GLfloat green, GLfloat blue )
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(90,1,1,100);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	gluLookAt(0, 1, 0, 0, 0, 0, 1, 0, 0);

	glPushMatrix();
	glColor3f(red,green,blue);
    glBegin(GL_QUADS);
	glVertex3f(100, -21, -100);
	glVertex3f(-100, -21, -100);
	glVertex3f(-100, -21, 100);
	glVertex3f(100, -21, 100);
	glEnd();
	glPopMatrix();

	point3DFloat_t position;
	position.x = -6;
	position.z = -3;
	position.y = -20;

	baseColor_t color;
	color.R = 1;
	color.G = 1;
	color.B = 0;
	DrawChar(position, drawChar, color);

}

void OPENGL_3D_ENGINE::DrawOverLookingView(GLfloat aspect)
{
	int serverEyeIdx = serverEyeBackBufIdx^1;
	int driverBufIdx = eyeBackBufIdx^1;

	//set the eye position and lookat position.
	setServerEyeOnce(serverEyeBuffer[serverEyeIdx].buffer[serverEyeBuffer[serverEyeIdx].index],aspect);
	//update the serverEye index		    
	if(serverEyeBuffer[serverEyeIdx].index < (serverEyeBuffer[serverEyeIdx].number-1))
	{
		serverEyeBuffer[serverEyeIdx].index += 1;
	}

	//use eye buffer to draw car
	if(eyeBuffer[driverBufIdx].number != 0)
	{
		//printf("index = %d\n",eyeBuffer[frontBufIdx].index);
		
		eyeLookAt_t eyeLookAtPosition = eyeBuffer[driverBufIdx].buffer[eyeBuffer[driverBufIdx].index];
			
	    GLfloat angle = ComputeAngle(&(eyeLookAtPosition.eyePosition), &(eyeLookAtPosition.lookatPosition));

		DrawCar(eyeBuffer[driverBufIdx].buffer[eyeBuffer[driverBufIdx].index].eyePosition, angle);
		//if server mode, it will mantain the car's position change
		if(ServerMode_3DEngine == drawMode)
		{
			if(eyeBuffer[driverBufIdx].index < (eyeBuffer[driverBufIdx].number-1))
			{
				eyeBuffer[driverBufIdx].index += 1;
			}
		}
	}
	//draw roads and signs
	DrawFrontBufferServer();
}

void OPENGL_3D_ENGINE::DrawDriverView(GLfloat aspect)
{
	int driverBufIdx = eyeBackBufIdx^1;
	//int charFrontBufIdx = charBackBufIdx^1;

	//point3DFloat_t charPosition;
	eyeLookAt_t eyeLookAtPosition = eyeBuffer[driverBufIdx].buffer[eyeBuffer[driverBufIdx].index];
	//set the eye position and lookat position.
	setClientEyeOnce(eyeLookAtPosition,ViewEnum_Driver,aspect);		
	//ComputePosAhead(&(eyeLookAtPosition.eyePosition), &(eyeLookAtPosition.lookatPosition), &charPosition);
	//draw all the charater
	//for(int charIdx = 0; charIdx < charBuffer[charFrontBufIdx].number; charIdx++)
	//{
	//	DrawChar(charPosition, charBuffer[charFrontBufIdx].buffer[charIdx].drawChar);
	//	charPosition.y++;
	//}

	if(eyeBuffer[driverBufIdx].index < (eyeBuffer[driverBufIdx].number-1))
	{
		eyeBuffer[driverBufIdx].index += 1;
	}

	//draw roads and signs
	DrawFrontBufferClient();
}

void OPENGL_3D_ENGINE::DrawLookAheadView(GLfloat aspect)
{
	int driverBufIdx = eyeLookaheadBackBufIdx^1;

	//set the eye position and lookat position.
	setClientEyeOnce(eyeBufferLookahead[driverBufIdx].buffer[eyeBufferLookahead[driverBufIdx].index],ViewEnum_LookAhead,aspect);

	if(eyeBufferLookahead[driverBufIdx].index < (eyeBufferLookahead[driverBufIdx].number-1))
	{
		eyeBufferLookahead[driverBufIdx].index += 1;
	}
	//draw roads and signs
	DrawFrontBufferClient();
}

void OPENGL_3D_ENGINE::DrawServerMode()
{
	int serverEyeIdx = serverEyeBackBufIdx^1;
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if(serverEyeBuffer[serverEyeIdx].number != 0)
	{
		glViewport(0,0,_winWidth,_winHeight);
		DrawOverLookingView((GLfloat)_winWidth/(GLfloat)_winHeight);
	}

	glutSwapBuffers();
}

void OPENGL_3D_ENGINE::DrawClientMode()
{
	int frontBufIdx = eyeBackBufIdx^1;
	int lookaheadBufIdx = eyeLookaheadBackBufIdx^1;
	int serverEyeIdx = serverEyeBackBufIdx^1;
	int LeftViewWidth = _winWidth;
	int LeftViewHeight = _winHeight;
	int RightViewHeight = 0;
	//printf("%d",frontBufIdx);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	if((serverEyeBuffer[serverEyeIdx].number !=0))
	{
		LeftViewWidth = _winWidth;
		LeftViewHeight = _winHeight;
		RightViewHeight  = _winHeight;
	}

	if((serverEyeBuffer[serverEyeIdx].number !=0)&&(eyeBuffer[frontBufIdx].number != 0))
	{
		LeftViewWidth = _winWidth/2;
		LeftViewHeight = _winHeight;
		RightViewHeight  = _winHeight;
	}

	if((serverEyeBuffer[serverEyeIdx].number !=0)&&(eyeBuffer[frontBufIdx].number != 0)&&
		(eyeBufferLookahead[lookaheadBufIdx].number != 0))
	{
		LeftViewWidth = _winWidth/2;
		LeftViewHeight = _winHeight;
		RightViewHeight  = _winHeight/2;
	}

	//draw server side veiwport
	//Driver view
	if(eyeBuffer[frontBufIdx].number != 0)
	{
		//printf("index = %d\n",eyeBuffer[frontBufIdx].index);
		glViewport(0,LeftViewHeight-CHAR_VIEW_HEIGHT,LeftViewWidth-SPLITE_LINE_WIDTH,CHAR_VIEW_HEIGHT);
	
	    int charFrontBufIdx = charBackBufIdx^1;
		int indexChar = 0;
		
		char showChar[100];
		memset(showChar, 0, 100);
		memcpy(showChar,(void*)"Driver View:  ",14);
		indexChar = 13;
		//charBuffer[charFrontBufIdx].buffer[0].drawChar
		if(charBuffer[charFrontBufIdx].number >0)
		{
			char* str = charBuffer[charFrontBufIdx].buffer[0].drawChar;
	        for(;*str!='\0';++str)
			{
				memcpy(&showChar[indexChar++],str,1);
			}
		}
		DrawCharView(showChar,121.0/255.0, 157.0/255.0, 248.0/255.0);

		glViewport(0,0,LeftViewWidth-SPLITE_LINE_WIDTH,LeftViewHeight-CHAR_VIEW_HEIGHT);
		DrawDriverView((GLfloat)(LeftViewWidth-SPLITE_LINE_WIDTH)/(GLfloat)(LeftViewHeight-CHAR_VIEW_HEIGHT));
	}

	//overlooking view
	if(serverEyeBuffer[serverEyeIdx].number != 0)
	{
		//draw splite line
		glViewport(LeftViewWidth,0,SPLITE_LINE_WIDTH,LeftViewHeight);
		DrawSpliteLine();

		glViewport(LeftViewWidth,RightViewHeight-SPLITE_LINE_WIDTH-CHAR_VIEW_HEIGHT,LeftViewWidth,CHAR_VIEW_HEIGHT);
		DrawCharView("OverLook View",192.0/255.0,176.0/255.0,143.0/255.0);

		//printf("index = %d\n",eyeBuffer[frontBufIdx].index);
		glViewport(LeftViewWidth,0,LeftViewWidth,RightViewHeight-SPLITE_LINE_WIDTH-CHAR_VIEW_HEIGHT);
		DrawOverLookingView((GLfloat)LeftViewWidth/(GLfloat)(RightViewHeight-SPLITE_LINE_WIDTH-CHAR_VIEW_HEIGHT));
	}

	if(eyeBufferLookahead[lookaheadBufIdx].number != 0)
	{
		//draw splite line
		glViewport(LeftViewWidth,RightViewHeight,LeftViewWidth,SPLITE_LINE_WIDTH);
		DrawSpliteLine();

		glViewport(LeftViewWidth,RightViewHeight+RightViewHeight-CHAR_VIEW_HEIGHT ,LeftViewWidth,CHAR_VIEW_HEIGHT);
		DrawCharView("Ahead View: 100M",121.0/255.0, 157.0/255.0, 248.0/255.0);

		//printf("index = %d\n",eyeBuffer[frontBufIdx].index);
		glViewport(LeftViewWidth,RightViewHeight,LeftViewWidth,RightViewHeight-CHAR_VIEW_HEIGHT);
		DrawLookAheadView((GLfloat)LeftViewWidth/(GLfloat)(RightViewHeight-CHAR_VIEW_HEIGHT));
	}

	glutSwapBuffers();
}

void OPENGL_3D_ENGINE::DrawAll()
{
	WaitForSingleObject(hMutex,INFINITE); 
	{
		if(ClientMode_3DEngine == drawMode)
		{
			DrawClientMode();
		}else if(ServerMode_3DEngine == drawMode)
		{
			DrawServerMode();
		}
	}
	ReleaseMutex(hMutex);

}

void OPENGL_3D_ENGINE::setWindow(int width, int height)
{
	_winWidth = width;
	_winHeight = height;
}

void OPENGL_3D_ENGINE::setSignFlag(void)
{
	_signFlag = (!_signFlag);
}


void OPENGL_3D_ENGINE::DrawCar(point3DFloat_t position, GLfloat angle)
{
	glPushMatrix();
	//red car
	
	//glTranslatef(position.x, 2.5, position.z);  //green car
	glTranslatef(position.x, 1, position.z);  //red car
    glRotatef(angle-90, 0,1,0);
	glScalef(0.0015,0.0015,0.0015);   //red car
	if(0 != CarShowList)
	{
		glCallList(CarShowList);
	}else
	{
		glColor3f(1.0f, 0.0f, 0.0f);
		glutSolidSphere(4, 40, 40);
	}
	glPopMatrix();
}

void OPENGL_3D_ENGINE::DrawSignServer(signInfo_t sign)
{
	GLint last_texture_ID;
	if((int)(sign.type) > MAX_BUFFER_DEPTH_2D_TXETURE)
	{
		return;
	}

    if(sign.attribute <= 0)
    {
        return;
    }

	glPushMatrix();

	//draw the sign on the road side
	glTranslatef(sign.position.x, 0.4, sign.position.z);
	glRotatef(-serverHeadAngle,0,1,0);
	glRotatef(5.0f, 0,0,1);

	float eyeHeight = serverEyeBuffer[serverEyeBackBufIdx^1].buffer->eyePosition.y;

	float ratio = (eyeHeight/150);
	ratio = ratio > 1?1:ratio;
	ratio = ratio < 0.1?0.1:ratio;

	GLfloat poleLength = sign.position.y*8*ratio;
	GLfloat poleWidth = ratio*0.3;
	DrawPoleServer(poleWidth, poleLength);

	//server mode, sign is forward the sky
	glTranslatef(poleLength+HALF_WIDTH_SIGN_OVERLOOKING*ratio, 0, 0);
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture_ID);
	glBindTexture(GL_TEXTURE_2D, texturelist[sign.type]);
	//glCallList(SignShowList_roadOn);
	{
		GLfloat
		Point1[] = {HALF_WIDTH_SIGN_OVERLOOKING*ratio, 0.11f, HALF_WIDTH_SIGN_OVERLOOKING*ratio},
		Point2[] = {-HALF_WIDTH_SIGN_OVERLOOKING*ratio, 0.11f, HALF_WIDTH_SIGN_OVERLOOKING*ratio},
		Point3[] = {-HALF_WIDTH_SIGN_OVERLOOKING*ratio, 0.11f, -HALF_WIDTH_SIGN_OVERLOOKING*ratio},
		Point4[] = {HALF_WIDTH_SIGN_OVERLOOKING*ratio,0.11f, -HALF_WIDTH_SIGN_OVERLOOKING*ratio};
	    GLfloat ColorR[] = {1, 0, 0,0.5};
		glBegin(GL_QUADS);
	
		// front side texture
		glTexCoord2f(1.0f, 1.0f);
		ColoredVertex(ColorR, Point1);
		glTexCoord2f(1.0f, 0.0f);
		ColoredVertex(ColorR, Point2);
		glTexCoord2f(0.0f, 0.0f);
		ColoredVertex(ColorR, Point3);
		glTexCoord2f(0.0f, 1.0f);
		ColoredVertex(ColorR, Point4);

		glEnd();
	}

	glBindTexture(GL_TEXTURE_2D, last_texture_ID);

	//glPopMatrix();

	//glPushMatrix();

	//add a flag number beside the sign
	char info[6];
	//int flag = sign.attribute>=5?5:sign.attribute;
	int showNum = sign.attribute;
	if(sign.attribute > 99)
	{
		showNum = 99;
	}
	if(showNum < 10)
	{
		sprintf(&info[0],"%d",showNum);
		memset(&info[1],0,1);
	}else
	{
		sprintf(&info[0],"%d",(showNum/10));
		sprintf(&info[1],"%d",(showNum%10));
		memset(&info[2],0,1);
	}

	//GLfloat red = 1-0.25*(flag-1);
	//GLfloat green = (flag-1)*0.25;
	glColor3f(0, 1, 0);

	glRasterPos3f(0, 0, (HALF_WIDTH_SIGN_OVERLOOKING+2)*ratio);
	//glRotatef(-serverHeadAngle,0,1,0);

	DrawCharWithOutPos(info);


	glPopMatrix();
}

void OPENGL_3D_ENGINE::DrawSignClient(signInfo_t sign)
{
	GLint last_texture_ID;
	if((int)(sign.type) > MAX_BUFFER_DEPTH_2D_TXETURE)
	{
		return;
	}

    if(sign.attribute <= 0)
    {
        return;
    }

	glPushMatrix();

	//draw the sign on the road side
	glTranslatef(sign.position.x, 0, sign.position.z);

	//client mode, sign is forward car perspective
	glRotatef(sign.rotAngle,0, 1, 0);
	DrawPole(sign.position.y - 0.5f);

	glTranslatef(0,sign.position.y,0);

	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture_ID);

	glBindTexture(GL_TEXTURE_2D, texturelist[sign.type]);
	{
		GLfloat
			Point1[] = {-0.5f, 0.5f, 0.11f},
			Point2[] = {0.5f,  0.5f, 0.11f},
			Point3[] = {0.5f, -0.5f, 0.11f},
			Point4[] = {-0.5f,-0.5f, 0.11f};
		GLfloat
			ColorR[] = {1, 0, 0,0.5};

		glBegin(GL_QUADS);
		// front side texture
		glTexCoord2f(0.0f, 1.0f);
		ColoredVertex(ColorR, Point1);
		glTexCoord2f(1.0f, 1.0f);
		ColoredVertex(ColorR, Point2);
		glTexCoord2f(1.0f, 0.0f);
		ColoredVertex(ColorR, Point3);
		glTexCoord2f(0.0f, 0.0f);
		ColoredVertex(ColorR, Point4);
		glEnd();
	}
	glBindTexture(GL_TEXTURE_2D, last_texture_ID);
	glCallList(SignShowList_roadSide);
	
	//server mode, sign is forward the sky
	glPopMatrix();
}

void OPENGL_3D_ENGINE::DrawSignOnRoad(signInfo_t sign)
{
	GLint last_texture_ID;
	if((int)(sign.type) > MAX_BUFFER_DEPTH_2D_TXETURE)
	{
		return;
	}

    if(sign.attribute <= 0)
    {
        return;
    }

	glPushMatrix();

	//draw the sign on the road side
	glTranslatef(sign.position.x, 0.01, sign.position.z);

	//client mode, sign is forward car perspective
	glRotatef((sign.rotAngle),0, 1, 0);

	glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture_ID);

	glBindTexture(GL_TEXTURE_2D, texturelist[sign.type]);
	{
		GLfloat
			Point1[] = {sign.sharp.vertex[0].x,sign.sharp.vertex[0].y,sign.sharp.vertex[0].z},//{-10.0f, 0.0f, 20.0f},
			Point2[] = {sign.sharp.vertex[1].x,sign.sharp.vertex[1].y,sign.sharp.vertex[1].z},//{10.0f,  0.0f, 20.0f},
			Point3[] = {sign.sharp.vertex[2].x,sign.sharp.vertex[2].y,sign.sharp.vertex[2].z},//{10.0f,  0.0f, -20.0f},
			Point4[] = {sign.sharp.vertex[3].x,sign.sharp.vertex[3].y,sign.sharp.vertex[3].z};//{-10.0f, 0.0f, -20.0f};
		GLfloat
			ColorR[] = {1, 0, 0, 0.5};

		glBegin(GL_QUADS);
		// front side texture
		glTexCoord2f(1.0f, 1.0f);
		ColoredVertex(ColorR, Point1);
		glTexCoord2f(0.0f, 1.0f);
		ColoredVertex(ColorR, Point2);
		glTexCoord2f(0.0f, 0.0f);
		ColoredVertex(ColorR, Point3);
		glTexCoord2f(1.0f, 0.0f);
		ColoredVertex(ColorR, Point4);
		glEnd();
	}

	glBindTexture(GL_TEXTURE_2D, last_texture_ID);
	
	glPopMatrix();
}

void OPENGL_3D_ENGINE::DrawPoleServer(GLfloat radius,GLfloat height)
{
#if 0
	GLfloat PointList[][3] = {
		0,  -0.3f,  -0.3f,
		0,  0.3f,  -0.3f,
		height,  -0.3f,  -0.3f,
		height,  0.3f,  -0.3f,

		0, -0.3f,  0.3f,
		0, 0.3f,  0.3f,
		height, -0.3f,  0.3f,
		height, 0.3f,  0.3f,
	};

	GLint index_list[][4] = {
		0,2,3,1,       //back side
		0,4,6,2,       //left side
		0,1,5,4,       //bottom side
		4,5,7,6,       //front side
		1,3,7,5,       //right side
		2,6,7,3,      //upper side
	};

	GLfloat ColorR[] = {0.5, 0.5, 0.5};

	int i,j;

	glBegin(GL_QUADS);
	glColor3fv(ColorR);
	for(i = 0; i < 6; i++)
	{
		for(j = 0; j < 4; j++)
		{
			glVertex3fv(PointList[index_list[i][j]]);
		}
	}
	glEnd();
#endif
	//GLfloat radius = 0.3;
	glColor3f(0.5f,0.5f,0.5f);
	glBegin(GL_QUAD_STRIP);
	for(int i=0; i<360; ++i)
	{
		glVertex3f( 0, radius*cos(2*PI/360*i)+0.11,radius*sin(2*PI/360*i));
		glVertex3f( height, radius*cos(2*PI/360*i)+0.11, radius*sin(2*PI/360*i));
	}
	glVertex3f(0, radius*cos(0.0)+0.11, radius*sin(0.0));
	glVertex3f(height, radius*cos(0.0)+0.11, radius*sin(0.0));
	glEnd();
}

void OPENGL_3D_ENGINE::DrawPole(GLfloat height)
{
#if 0
	GLfloat PointList[][3] = {
		-0.1f,  0,  -0.01f,
		0.1f,  0,  -0.01f,
		-0.1f,  height,  -0.01f,
		0.1f,  height,  -0.01f,

		-0.1f, 0,  0.01f,
		0.1f, 0,  0.01f,
		-0.1f, height,  0.01f,
		0.1f, height,  0.01f,
	};

	GLint index_list[][4] = {
		0,2,3,1,       //back side
		0,4,6,2,       //left side
		0,1,5,4,       //bottom side
		4,5,7,6,       //front side
		1,3,7,5,       //right side
		2,6,7,3,      //upper side
	};

	GLfloat ColorR[] = {0.5, 0.5, 0.5};

	int i,j;

	glBegin(GL_QUADS);
	glColor3fv(ColorR);
	for(i = 0; i < 6; i++)
	{
		for(j = 0; j < 4; j++)
		{
			glVertex3fv(PointList[index_list[i][j]]);
		}
	}
	glEnd();
#endif

	GLfloat radius = 0.1;
	glColor3f(0.5f,0.5f,0.5f);
	glBegin(GL_QUAD_STRIP);
	for(int i=0; i<360; ++i)
	{
		glVertex3f( radius*cos(2*PI/360*i), 0, radius*sin(2*PI/360*i));
		glVertex3f( radius*cos(2*PI/360*i), height, radius*sin(2*PI/360*i));
	}
	glVertex3f(radius*cos(0.0), 0, radius*sin(0.0));
	glVertex3f(radius*cos(0.0), height, radius*sin(0.0));
	glEnd();

}

void OPENGL_3D_ENGINE::DrawQuad(quadInfo_t quad)
{
	//glBegin(GL_QUADS);
	glBegin(GL_TRIANGLE_STRIP);
	//glColor3f(1.0f,0.95f,0.9f);  //white
	glColor3f(quad.color.R,quad.color.G,quad.color.B);
	glVertex3f(quad.vertex[0].x,quad.vertex[0].y,quad.vertex[0].z);
	glVertex3f(quad.vertex[1].x,quad.vertex[1].y,quad.vertex[1].z);
	glVertex3f(quad.vertex[2].x,quad.vertex[2].y,quad.vertex[2].z);
	glVertex3f(quad.vertex[3].x,quad.vertex[3].y,quad.vertex[3].z);
	glVertex3f(quad.vertex[0].x,quad.vertex[0].y,quad.vertex[0].z);
	glEnd();
}

void OPENGL_3D_ENGINE::DrawLine(lineInfo_t *line,mode3DEngineEnum_t type)
{
	int index = 1;
	glPushMatrix();

	//dash line
	if(lineTypeEnum_dash == line->type)
	{
		if(ClientMode_3DEngine == type)
		{
			//glLineWidth(4.0f);
			glBegin(GL_QUADS);
			//glColor3f(1.0f,0.95f,0.9f);  //white
			glColor3f(line->color.R, line->color.G, line->color.B);
			int step = 6;
			GLfloat offset = 0.1;
			for(index = 0; index < ((line->position.size())-1); index++)
			{
				//compute step
				GLfloat total_x = (line->position[index+1].x - line->position[index].x);
				GLfloat total_z = (line->position[index+1].z - line->position[index].z);
				GLfloat distance = sqrt(total_x*total_x + total_z*total_z);
				int temp_step = (int)((distance+0.5)/1.5);
				step = (temp_step/2)*2;
				if(step == 0)
					continue;

				for(int index2 = 0; index2 < step; index2++)
				{
					GLfloat angle = ComputeAngle(&line->position[index], &line->position[index+1]);

					GLfloat det_x = (line->position[index+1].x - line->position[index].x)/step;
					GLfloat det_z = (line->position[index+1].z - line->position[index].z)/step;
					int direction = -1;
					if((index2&0x1) == 0)
					{direction = 1;}
					glVertex3f(line->position[index].x+index2*det_x-direction*offset*sin(-angle*PI/180), 
						line->position[index].y+0.0001, 
						line->position[index].z+index2*det_z+direction*offset*cos(-angle*PI/180));
					glVertex3f(line->position[index].x+index2*det_x+direction*offset*sin(-angle*PI/180), 
						line->position[index].y+0.0001, 
						line->position[index].z+index2*det_z-direction*offset*cos(-angle*PI/180));
				}
			}
			glEnd();
		}else if(ServerMode_3DEngine == type)
		{
#if 0
			glLineWidth(0.5f);
			glBegin(GL_LINES);
			glColor3f(1.0f,0.95f,0.9f);  //white
			int step = 2;
			for(index = 0; index < ((line->number)-1); index++)
			{
				for(int index2 = 0; index2 < step; index2++)
				{
					GLfloat det_x = (line->position[index+1].x - line->position[index].x)/step;
					GLfloat det_z = (line->position[index+1].z - line->position[index].z)/step;
					glVertex3f(line->position[index].x+index2*det_x, line->position[index].y+0.01, line->position[index].z+index2*det_z);
				}
			}
			glEnd();
#else
			glBegin(GL_QUADS);
			//glColor3f(1.0f,0.95f,0.9f);  //white
			glColor3f(line->color.R, line->color.G, line->color.B);
			int step = 6;
			GLfloat offset = 0.1;
			for(index = 0; index < ((line->position.size())-1); index++)
			{
				//compute step
				GLfloat total_x = (line->position[index+1].x - line->position[index].x);
				GLfloat total_z = (line->position[index+1].z - line->position[index].z);
				GLfloat distance = sqrt(total_x*total_x + total_z*total_z);
				int temp_step = (int)((distance+0.5)/2);
				step = (temp_step/2)*2;
				if(step == 0)
					continue;

				for(int index2 = 0; index2 < step; index2++)
				{
					GLfloat angle = ComputeAngle(&line->position[index], &line->position[index+1]);

					GLfloat det_x = (line->position[index+1].x - line->position[index].x)/step;
					GLfloat det_z = (line->position[index+1].z - line->position[index].z)/step;
					int direction = -1;
					if((index2&0x1) == 0)
					{direction = 1;}
					glVertex3f(line->position[index].x+index2*det_x-direction*offset*sin(-angle*PI/180), 
						line->position[index].y+0.01, 
						line->position[index].z+index2*det_z+direction*offset*cos(-angle*PI/180));
					glVertex3f(line->position[index].x+index2*det_x+direction*offset*sin(-angle*PI/180), 
						line->position[index].y+0.01, 
						line->position[index].z+index2*det_z-direction*offset*cos(-angle*PI/180));
				}
			}
			glEnd();
#endif
		}
	}else if(lineTypeEnum_solid == line->type)
	{
		//glEnable(GL_BLEND);
		//glEnable(GL_LINE_SMOOTH);
		//glHint(GL_LINE_SMOOTH_HINT, GL_FASTEST);
		//glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); 
		if(ClientMode_3DEngine == type){
		glLineWidth(2.0f);
		}else
		{glLineWidth(1.0f);}
		glBegin(GL_LINE_STRIP);
		//set the color of line
		//glColor3f(1.0f,0.95f,0.9f);  //white
		glColor3f(line->color.R, line->color.G, line->color.B);
		for(index = 0; index < line->position.size(); index++)
		{
			glVertex3f(line->position[index].x, line->position[index].y+0.005, line->position[index].z);
		}
		glEnd();
		//glDisable(GL_BLEND);
	}

	glPopMatrix();
}
void OPENGL_3D_ENGINE::DrawCharDynamic(drawCharInfo_t input)
{
	char* str = input.drawChar;
	HDC hDC = wglGetCurrentDC();
	GLuint list = glGenLists(1);

	glPushMatrix();

	glColor3f(1.0f, 1.0f, 0.0f);
	glRasterPos3f(input.position.x,input.position.y,input.position.z);
	//glRasterPos2f(input.position.x,input.position.y);

	for(;*str!='\0';++str)
	{
		wglUseFontBitmaps(hDC, *str, 1, list);
		glCallList(list);
	}
	glPopMatrix();
	glDeleteLists(list,1);
}

void OPENGL_3D_ENGINE::DrawChar(point3DFloat_t position, char *drawChar, baseColor_t color)
{
	char* str = drawChar;
	glPushMatrix();

	glColor3f(color.R, color.G, color.B);
	glRasterPos3f(position.x,position.y,position.z);

	//glScalef(0.1,0.1,0.1);
	for(;*str!='\0';++str)
		glCallList(CharShowList + *str);
	glPopMatrix();
}

void OPENGL_3D_ENGINE::DrawCharWithOutPos(char *drawChar)
{
	char* str = drawChar;
	for(;*str!='\0';++str)
		glCallList(CharShowList + *str);
}

void OPENGL_3D_ENGINE::DrawRoadSide(lineInfo_t* line1)
{
	int index;
	if(lineTypeEnum_roadside_line == line1->type)
	{
		glColor3f(0.2f,0.2f,0.2f);
		glBegin(GL_QUADS);
		for(index = 0; index < (line1->position.size()-1); index++)
		{
			glVertex3f(line1->position[index].x,line1->position[index].y,line1->position[index].z);
			glVertex3f(line1->position[index+1].x,line1->position[index+1].y,line1->position[index+1].z);
			glVertex3f(line1->position[index+1].x,line1->position[index+1].y+0.3,line1->position[index+1].z);
			glVertex3f(line1->position[index].x,line1->position[index].y+0.3,line1->position[index].z);
		}
		glEnd();
	}
}

void OPENGL_3D_ENGINE::DrawBlueSky()
{
	//glColor3f(71.0/255.0, 111.0/255.0, 184/255.0);
	glColor3f(128.0/255.0, 160.0/255.0, 250.0/255.0);
	//glColor4f(128.0/255.0, 160.0/255.0, 250.0/255.0,1.0f);
	glBegin(GL_QUAD_STRIP);
	for(int i=0; i<360; ++i)
	{
		glVertex3f(1800*cos(2*PI/360*i), 0, 1800*sin(2*PI/360*i));
		glVertex3f(1800*cos(2*PI/360*i), 450, 1800*sin(2*PI/360*i));
	}
	glVertex3f(1800*cos(0.0), 0, 1800*sin(0.0));
	glVertex3f(1800*cos(0.0), 450, 1800*sin(0.0));
	glEnd();


	glColor3f(124.0/255.0, 159.0/255.0, 249.0/255.0);
	//glColor4f(124.0/255.0, 159.0/255.0, 249.0/255.0,1.0f);
	glBegin(GL_QUAD_STRIP);
	for(int i=0; i<360; ++i)
	{
		glVertex3f(1800*cos(2*PI/360*i), 450, 1800*sin(2*PI/360*i));
		glVertex3f(1800*cos(2*PI/360*i), 850, 1800*sin(2*PI/360*i));
	}
	glVertex3f(1800*cos(0.0), 450, 1800*sin(0.0));
	glVertex3f(1800*cos(0.0), 850, 1800*sin(0.0));
	glEnd();

	glColor3f(121.0/255.0, 157.0/255.0, 248.0/255.0);
	//glColor4f(121.0/255.0, 157.0/255.0, 248.0/255.0,1.0f);
	glBegin(GL_QUAD_STRIP);
	for(int i=0; i<360; ++i)
	{
		glVertex3f(1800*cos(2*PI/360*i), 850, 1800*sin(2*PI/360*i));
		glVertex3f(1800*cos(2*PI/360*i), 2000, 1800*sin(2*PI/360*i));
	}
	glVertex3f(1800*cos(0.0), 850, 1800*sin(0.0));
	glVertex3f(1800*cos(0.0), 2000, 1800*sin(0.0));
	glEnd();
}

void OPENGL_3D_ENGINE::DrawRoadwithLine(lineInfo_t* line1, lineInfo_t* line2)
{
	//find the min number of the two line
	int stripNum = (line1->position.size()) > (line2->position.size()) ?(line2->position.size()):(line1->position.size());
	int index = 0;
    //GLint last_texture_ID;

	//server mode, sign is forward the sky
	//glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture_ID);
	//glBindTexture(GL_TEXTURE_2D, texturelist[ROAD_SURFACE_TEXTURE_IDX]);

	//glColor3f(0.25f,0.25f,0.25f);
	glColor3f(line1->color.R, line1->color.G, line1->color.B);

	glBegin(GL_TRIANGLE_STRIP);

	for(index = 0; index < stripNum; index++)
	{
		//glTexCoord2f(0.0f, (GLfloat)(index&0x1));
		glVertex3f(line1->position[index].x,line1->position[index].y,line1->position[index].z);
		//glTexCoord2f(1.0f, (GLfloat)(index&0x1));
		glVertex3f(line2->position[index].x,line2->position[index].y,line2->position[index].z);
	}
	glEnd();

    glBegin(GL_TRIANGLE_FAN);
	if((line1->position.size()) > (line2->position.size()))
	{
		glVertex3f(line2->position[stripNum-1].x,line2->position[stripNum-1].y,line2->position[stripNum-1].z);
		for(index = (stripNum-1); index < (line1->position.size()); index++)
		{
			glVertex3f(line1->position[index].x,line1->position[index].y,line1->position[index].z);
		}
	}

	if((line1->position.size()) < (line2->position.size()))
	{
		glVertex3f(line1->position[stripNum-1].x,line1->position[stripNum-1].y,line1->position[stripNum-1].z);
		for(index = (stripNum-1); index < (line2->position.size()); index++)
		{
			glVertex3f(line2->position[index].x,line2->position[index].y,line2->position[index].z);
		}
	}
	glEnd();

	//glBindTexture(GL_TEXTURE_2D, last_texture_ID);

}


void OPENGL_3D_ENGINE::CreateSignRoadOnShowList()
{
	GLfloat
		Point1[] = {HALF_WIDTH_SIGN_OVERLOOKING, 0.4f, HALF_WIDTH_SIGN_OVERLOOKING},
		Point2[] = {-HALF_WIDTH_SIGN_OVERLOOKING, 0.4f, HALF_WIDTH_SIGN_OVERLOOKING},
		Point3[] = {-HALF_WIDTH_SIGN_OVERLOOKING, 0.4f, -HALF_WIDTH_SIGN_OVERLOOKING},
		Point4[] = {HALF_WIDTH_SIGN_OVERLOOKING,0.4f, -HALF_WIDTH_SIGN_OVERLOOKING};
	GLfloat
		ColorR[] = {1, 0, 0,0.5};

	SignShowList_roadOn = glGenLists(1);

	glNewList(SignShowList_roadOn, GL_COMPILE);
	glBegin(GL_QUADS);
	
    // front side texture
	glTexCoord2f(1.0f, 1.0f);
	ColoredVertex(ColorR, Point1);
	glTexCoord2f(1.0f, 0.0f);
	ColoredVertex(ColorR, Point2);
	glTexCoord2f(0.0f, 0.0f);
	ColoredVertex(ColorR, Point3);
	glTexCoord2f(0.0f, 1.0f);
	ColoredVertex(ColorR, Point4);

	glEnd();
	glEndList();
}

void OPENGL_3D_ENGINE::CreateSignRoadSideShowList()
{
	GLfloat
		Point1[] = {-0.5f, 0.5f, 0.11f},
        Point2[] = {0.5f,  0.5f, 0.11f},
        Point3[] = {0.5f, -0.5f, 0.11f},
        Point4[] = {-0.5f,-0.5f, 0.11f},
        Point5[] = {-0.5f, 0.5f, -0.01f},
        Point6[] = {0.5f,  0.5f, -0.01f},
        Point7[] = {0.5f, -0.5f, -0.01f},
        Point8[] = {-0.5f,-0.5f, -0.01f};
	GLfloat
		ColorR[] = {1, 0, 0,0.5},
		ColorY[] = {0.0, 0.0, 0.0},
		ColorZ[] = {0.5,0.5,0.5};

	SignShowList_roadSide = glGenLists(1);

	glNewList(SignShowList_roadSide, GL_COMPILE);
	glBegin(GL_QUADS);
	// front side texture
	//glTexCoord2f(0.0f, 1.0f);
	ColoredVertex(ColorZ, Point1);
	//glTexCoord2f(1.0f, 1.0f);
	ColoredVertex(ColorZ, Point2);
	//glTexCoord2f(1.0f, 0.0f);
	ColoredVertex(ColorZ, Point3);
	//glTexCoord2f(0.0f, 0.0f);
	ColoredVertex(ColorZ, Point4);

	// back side
	ColoredVertex(ColorY, Point5);
	ColoredVertex(ColorY, Point6);
	ColoredVertex(ColorY, Point7);
	ColoredVertex(ColorY, Point8);

	// left and right side
	ColoredVertex(ColorZ, Point1);
	ColoredVertex(ColorZ, Point4);
	ColoredVertex(ColorZ, Point8);
	ColoredVertex(ColorZ, Point5);

	ColoredVertex(ColorZ, Point1);
	ColoredVertex(ColorZ, Point2);
	ColoredVertex(ColorZ, Point6);
	ColoredVertex(ColorZ, Point5);

	ColoredVertex(ColorZ, Point2);
	ColoredVertex(ColorZ, Point3);
	ColoredVertex(ColorZ, Point7);
	ColoredVertex(ColorZ, Point6);

	ColoredVertex(ColorZ, Point3);
	ColoredVertex(ColorZ, Point4);
	ColoredVertex(ColorZ, Point8);
	ColoredVertex(ColorZ, Point7);

	glEnd();
	glEndList();
}

void OPENGL_3D_ENGINE::CreateCharaterShowList()
{
	//set the charater size and type
	HFONT hFont = CreateFontA(20, 0, 0, 0, FW_MEDIUM, 0, 0, 0,
	ANSI_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
	DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, "Comic Sans MS");
	HFONT hOldFont = (HFONT)SelectObject(wglGetCurrentDC(), hFont);
	DeleteObject(hOldFont);

	CharShowList = glGenLists(128);
	wglUseFontBitmaps(wglGetCurrentDC(), 0, 128, CharShowList);
}

void OPENGL_3D_ENGINE::selectFont(int size,  int charset)
{
	//set the charater size and type
	HFONT hFont = CreateFontA(size, 0, 0, 0, FW_MEDIUM, 0, 0, 0,
	charset, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
	DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, "Comic Sans");
	HFONT hOldFont = (HFONT)SelectObject(wglGetCurrentDC(), hFont);
	DeleteObject(hOldFont);
}

int OPENGL_3D_ENGINE::power_of_two(int n)
{
	if( n <= 0 )
		return 0;
	return (n & (n-1)) == 0;
}

//
GLboolean OPENGL_3D_ENGINE::load_bmp24_texture(const char* file_name, int type)
{
	GLint width, height, total_bytes;
	GLubyte* pixels = 0;
	GLuint last_texture_ID, texture_ID = 0;

	// open the file, if failed, reture
	FILE* pFile = fopen(file_name, "rb");
	if( pFile == 0 )
		return GL_FALSE;

	if(type > MAX_BUFFER_DEPTH_2D_TXETURE)
	{
		return GL_FALSE;
	}
    //read the file image width and height.
	fseek(pFile, 0x0012, SEEK_SET);
	fread(&width, 4, 1, pFile);
	fread(&height, 4, 1, pFile);
	fseek(pFile, BMP_Header_Length, SEEK_SET);

    //compute how many bytes in each line, and compute the total pixel bytes.
	{
		GLint line_bytes = width * 3;
		while( line_bytes % 4 != 0 )
			++line_bytes;
		total_bytes = line_bytes * height;
	}

    //allocate the memory for pixel image bytes
	pixels = (GLubyte*)malloc(total_bytes);
	if( pixels == 0 )
	{
		fclose(pFile);
		return GL_FALSE;
	}

    //read pixel data
	if( fread(pixels, total_bytes, 1, pFile) <= 0 )
	{
		free(pixels);
		fclose(pFile);
		return GL_FALSE;
	}

    //if the image width and height is not a power of integer, need to do zoom.
	{
		GLint max;
		glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max);
		if( !power_of_two(width)
			|| !power_of_two(height)
			|| width > max
			|| height > max )
		{
			const GLint new_width = 256;
			const GLint new_height = 256; 
			GLint new_line_bytes, new_total_bytes;
			GLubyte* new_pixels = 0;

            //compute the bytes number for each line and total image 
			new_line_bytes = new_width * 3;
			while( new_line_bytes % 4 != 0 )
				++new_line_bytes;
			new_total_bytes = new_line_bytes * new_height;

			// allocate the the new memory
			new_pixels = (GLubyte*)malloc(new_total_bytes);
			if( new_pixels == 0 )
			{
				free(pixels);
				fclose(pFile);
				return GL_FALSE;
			}

            //zoom in/out pixel data
			gluScaleImage(GL_RGB,
				width, height, GL_UNSIGNED_BYTE, pixels,
				new_width, new_height, GL_UNSIGNED_BYTE, new_pixels);

            //release original pixel data and reset the width and height
			free(pixels);
			pixels = new_pixels;
			width = new_width;
			height = new_height;
		}
	}

    //allocate a new texture number
	glGenTextures(1, &texture_ID);
	if( texture_ID == 0 )
	{
		free(pixels);
		fclose(pFile);
		return GL_FALSE;
	}

	// bind the new texture file
    //before bind, get the original texture number to restore
	glGetIntegerv(GL_TEXTURE_BINDING_2D, (GLint*)&last_texture_ID);
	glBindTexture(GL_TEXTURE_2D, texture_ID);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0,
		GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);
	glBindTexture(GL_TEXTURE_2D, last_texture_ID);

	// release memory after glTexImage2D, OpenGL has save pixels data
	free(pixels);

	texturelist[type] = texture_ID;
	return GL_TRUE;
}

GLfloat OPENGL_3D_ENGINE::ComputeAngle(point3DFloat_t *startPoint, point3DFloat_t *endPoint)
{
    //compute x,z coordinate's angle
    GLfloat angle = atan2((endPoint->z - startPoint->z),(endPoint->x - startPoint->x));
    return -180*atan2((endPoint->z - startPoint->z),(endPoint->x - startPoint->x))/PI;

}

void OPENGL_3D_ENGINE::loadCarModle(const char* file_name)
{
   Loader.OpenFile(file_name);
   Loader.LoadFile();
   Loader.CloseFile();

   CreateCarShowList();
}

void OPENGL_3D_ENGINE::CreateCarShowList(void)
{
	CarShowList = glGenLists( 1 );
	glNewList( CarShowList, GL_COMPILE );

	Model TempModel = Loader.GetModel();
	for( size_t i = 0; i != TempModel.MyObject.size(); ++ i )
	{
	   const Object& object = TempModel.MyObject[i];
	   glBegin( GL_TRIANGLES ); 
	   for(int j = 0; j != object.Faces.size(); j++)
	   {
		const Face& ThisFace = object.Faces[ j ];
		const Material& MyMaterial = TempModel.MyMaterial[ ThisFace.MaterialPos ];

		glNormal3f( ThisFace.Normal.x, ThisFace.Normal.y, ThisFace.Normal.z );
		glColor4f( MyMaterial.diffuseColor[ 0 ], MyMaterial.diffuseColor[ 1 ],
				   MyMaterial.diffuseColor[ 2 ], MyMaterial.transparency );

		if( MyMaterial.transparency )
		 glEnable( GL_BLEND );
		for( size_t k = 0; k != 3; ++ k )
		{
		 size_t index = object.Faces[ j ].Index[ k ];
		 glVertex3f( object.Vertexs[ index ].x, object.Vertexs[ index ].y, object.Vertexs[ index ].z );
		}
		if( MyMaterial.transparency )
		 glDisable( GL_BLEND );
	   }
	   glEnd();
	}
	glEndList();
}