/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  Visualization.h
* @brief A sample 3D engine, provide some function to add road, sign, and perspective information.
*        road and sign information can load in once or each seconds.
*        perspective information is based one road and sign coordinates, each frame has its own information.
*
* Change Log:
*      Date                Who             What
*      2014/12/23         Xin Shao        Create
*******************************************************************************
*/
#pragma once

#include <gl/glut.h>
#include <windows.h>
#include <vector>
#include <list>
#include "Loader3DS.h"

using namespace std;

#define PI  3.1415926536f
#define FRAME_NUM_PER_SECOND  10     //the frame speed per second to draw
#define WIDTH_SIGN_OVERLOOKING 10    //the width in meters in overlooking perspective
#define WIDTH_SIGN_DRIVER      1     //the width in meters in driver perspective
#define HALF_WIDTH_SIGN_OVERLOOKING (WIDTH_SIGN_OVERLOOKING/2) 
#define HALF_WIDTH_SIGN_DRIVER      (WIDTH_SIGN_DRIVER/2)
#define Driver_VIEW_OFFSET          (0.0)
#define CAR_POS_OFFSET              (0.0)

#define ROAD_SURFACE_TEXTURE_IDX    1000

//how many Pixels points used for splite line width
#define SPLITE_LINE_WIDTH     2
#define CHAR_VIEW_HEIGHT      30

//set the road information, you can set 100 meters road of before or set mare meters road
#define NEWCO_STRUCTURE_PACK

//define the bmp 24bit picture's header length
#define BMP_Header_Length 54

#define MAX_BUFFER_NUMBER_3D_ENGINE  2      //only support 2 buffer now
#define MAX_BUFFER_DEPTH_3D_ENGINE   1000
#define MAX_BUFFER_DEPTH_2D_TXETURE  1024
#define MAX_BUFFER_LENGTH_DRAW_CHAR  100
#define MAX_BUFFER_DEPTH_DRAW_CHAR   10
//#define MAX_BUFFER_DEPTH_DRAW_LINE_POINT 5000
//#define MAX_BUFFER_NUMBER_DRAW_LINE   300
//#define MAX_BUFFER_NUMBER_DRAW_QUAD   200000

typedef struct
{
	float R;
	float G;
	float B;
	float A;
}baseColor_t;

typedef struct
{
	int x;
	int y;
	int width;
	int height;
}windowViewPort_t;

typedef enum
{
	lineTypeEnum_invalid = 0,  //reference line
	lineTypeEnum_dash    = 1,
	lineTypeEnum_solid   = 2,
	lineTypeEnum_double_solid = 3,
	lineTypeEnum_road_line = 4,
	lineTypeEnum_roadside_line = 5,
	lineTypeEnum_t_eMax
}lineTypeEnum_t;

typedef enum
{
	ClientMode_3DEngine = 0,
	ServerMode_3DEngine
}mode3DEngineEnum_t;

typedef enum
{
	ViewEnum_LookAhead = 0,
	ViewEnum_Driver
}modeViewEnum_t;

typedef struct NEWCO_STRUCTURE_PACK
{
	GLfloat x;
	GLfloat y;
	GLfloat z;    
}point3DFloat_t;

typedef struct NEWCO_STRUCTURE_PACK
{
	point3DFloat_t eyePosition;
	point3DFloat_t lookatPosition;
}eyeLookAt_t;

typedef struct NEWCO_STRUCTURE_PACK
{
	int type;
	GLfloat rotAngle;    //each sign's rotation angle,unit is degree,positive is right rotation, rangle:[-180, 180]
	point3DFloat_t position;
	int attribute;
}signInfo_t;

typedef struct NEWCO_STRUCTURE_PACK
{
	lineTypeEnum_t type;    //type of line, dash line, solid line, double solid line, etc
	//int number;  //how many point it has
	baseColor_t color;
	//point3DFloat_t position[MAX_BUFFER_DEPTH_DRAW_LINE_POINT];
	vector<point3DFloat_t> position;
}lineInfo_t;

typedef struct
{
	point3DFloat_t vertex[4];
	baseColor_t    color;
}quadInfo_t;

/*
typedef struct
{
	int number;
	quadInfo_t buffer[MAX_BUFFER_NUMBER_DRAW_QUAD];
}quadInfo_3D_Internal_t;
*/

//typedef struct NEWCO_STRUCTURE_PACK
//{
//	int number;
//	signInfo_t buffer[MAX_BUFFER_DEPTH_3D_ENGINE];
//}signInfo_3D_Internal_t;

typedef struct NEWCO_STRUCTURE_PACK
{
	int number;
	int index;        //current to use which eyeLookAt structure
	eyeLookAt_t buffer[MAX_BUFFER_DEPTH_3D_ENGINE];
}eyeInfo_3D_Internal_t;

typedef struct NEWCO_STRUCTURE_PACK
{
	point3DFloat_t position;
	char drawChar[MAX_BUFFER_LENGTH_DRAW_CHAR];
}drawCharInfo_t;

typedef struct NEWCO_STRUCTURE_PACK
{
	int number;
	drawCharInfo_t buffer[MAX_BUFFER_DEPTH_DRAW_CHAR];
}drawCharInfo_3D_Internal_t;


class OPENGL_3D_ENGINE
{
public:
	OPENGL_3D_ENGINE();
	~OPENGL_3D_ENGINE();

	//this function to add Sign inforation to sample 3D engine, it will copy the sign information to 3D engine's backend internal buffer.
	GLboolean AddSignInfo(vector<signInfo_t>& buffer);  //add road side Sign inforamtion 

	//this function to add one line information to sample 3D engine
	GLboolean AddOneLineInfo(lineTypeEnum_t type, baseColor_t color, vector<point3DFloat_t>& buffer);

	//this function to add one road side line information to sampel 3D engine
	GLboolean AddOneRoadLineInfo(lineTypeEnum_t type, baseColor_t color, vector<point3DFloat_t>& buffer);

	GLboolean AddQuadInfo(int number, quadInfo_t* buffer);

	//change the backend buffer to front buffer to draw, it means add roadInfomation and Sign information finished.
	GLboolean Swap3DBuffers(void);

	//change the backend Sign information(road furniture) buffer
	GLboolean SwapSignBuffer(void);

	//change the backend line buffer
	GLboolean SwapLineBuffer(void);

    //change the backend road line buffer
	GLboolean SwapRoadLineBuffer(void);

	//change the eye look at backend buffer to front buffer to draw.
	GLboolean SwapEyeBuffer(void);

	GLboolean SwapLookaheadEyeBuffer(void);

	//change the server eye look at backend buffer to front buffer to draw.
	GLboolean SwapServerEyeBuffer(void);

	//change the draw char backend buffer to front buffer to draw
	GLboolean SwapCharBuffer(void);

	GLboolean SwapQuadBuffer(void);

	//we don't set the head direction, default head direction is (0,1,0) ,  it will add the eyeBuffer to 3D engine's backend internal buffer.
	GLboolean setEyeLookat(int number,           //how many frames will draw based on the following information, this function is based one road inforamtion coordinates
		eyeLookAt_t* buffer);

	GLboolean setServerEyeLookat(int number, eyeLookAt_t* buffer);

	GLboolean setLookAheadEyeLookat(int number, eyeLookAt_t* buffer);

	//set a drawed character information,only support english charater, when finished add character information, pls call SwapCharBuffer() to show 
	GLboolean setDrawChar(drawCharInfo_t *buffer);


	void DrawAll();
	void setWindow(int width, int height);
	//load 24 bit bmp picture as type, the input bmp must be 256x256, after loading, signInfo_t's sign field must match to 
	// this function's type value. 
	GLboolean load_bmp24_texture(const char* file_name, int type);

	void setMode(mode3DEngineEnum_t mode);
	mode3DEngineEnum_t getMode(void);

	void setServerAngle(GLfloat angle);
	GLfloat getServerAngle(void);
	void loadCarModle(const char* file_name);

private:
	void DrawSignServer(signInfo_t sign);
	void DrawSignClient(signInfo_t sign);
	void DrawPole(GLfloat height = 2.0f);
	void DrawPoleServer(GLfloat height = 20.0f);
	void DrawCharDynamic(drawCharInfo_t input);
	void DrawChar(point3DFloat_t position, char *drawChar);
	void DrawCharWithOutPos(char *drawChar);
	void DrawLine(lineInfo_t *line,mode3DEngineEnum_t type);
	void DrawCar(point3DFloat_t position,GLfloat angle);
	void DrawFrontBufferClient();
	void DrawFrontBufferServer();
	void DrawRoadwithLine(lineInfo_t* line1, lineInfo_t* line2);
	void DrawRoadSide(lineInfo_t* line1);
	void DrawBlueSky();
	void DrawQuad(quadInfo_t quad);

	void DrawSpliteLine();
	void DrawCharView(char *drawChar,GLfloat red, GLfloat green, GLfloat blue);

	void DrawOverLookingView(GLfloat aspect);
	void DrawDriverView(GLfloat aspect);
	void DrawLookAheadView(GLfloat aspect);

	void DrawClientMode();
	void DrawServerMode();

	void setClientEyeOnce(eyeLookAt_t eye, modeViewEnum_t mode, GLfloat aspect);
	void setServerEyeOnce(eyeLookAt_t eye, GLfloat aspect);

	int  power_of_two(int n);
	void CreateSignRoadSideShowList();
	void CreateSignRoadOnShowList();
	void CreateCharaterShowList();
	void CreateCarShowList(void);
	GLfloat ComputeAngle(point3DFloat_t *startPoint, point3DFloat_t *endPoint);

	//int backBufIdx;         //backend buffer index
	int eyeBackBufIdx;      //eye backend buffer index
	int charBackBufIdx;     //draw charater backend buffer index
	int lineBackBufIdx;
	int signBackbufIdx;
	int roadLineBackBufIdx;
	int serverEyeBackBufIdx;
	int eyeLookaheadBackBufIdx;
	int quadBackBufIdx;

	
	eyeInfo_3D_Internal_t  eyeBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	eyeInfo_3D_Internal_t  eyeBufferLookahead[MAX_BUFFER_NUMBER_3D_ENGINE];
	eyeInfo_3D_Internal_t  serverEyeBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	vector<signInfo_t> signBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	list<lineInfo_t> lineBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	list<lineInfo_t> roadLineBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	drawCharInfo_3D_Internal_t charBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	vector<quadInfo_t>     quadBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	GLuint                 texturelist[MAX_BUFFER_DEPTH_2D_TXETURE];

	int SignShowList_roadSide;
	int SignShowList_roadOn;
	int CharShowList;
	int CarShowList;
	int _winWidth;
	int _winHeight;
	HANDLE hMutex;
	mode3DEngineEnum_t drawMode;
	GLfloat serverHeadAngle;
	Loader3DS Loader;

};

