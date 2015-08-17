#pragma once

#include <gl/glut.h>
#include <windows.h>

//set the road information, you can set 100 meters road of before or set mare meters road
#define NEWCO_STRUCTURE_PACK

//define the bmp 24bit picture's header length
#define BMP_Header_Length 54

#define MAX_BUFFER_NUMBER_3D_ENGINE  2      //only support 2 buffer now
#define MAX_BUFFER_DEPTH_3D_ENGINE   1000
#define MAX_BUFFER_DEPTH_2D_TXETURE  1024
#define MAX_BUFFER_LENGTH_DRAW_CHAR  100
#define MAX_BUFFER_DEPTH_DRAW_CHAR   10
#define MAX_BUFFER_DEPTH_DRAW_LINE_POINT 1024
#define MAX_BUFFER_NUMBER_DRAW_LINE   24

typedef enum
{
	lineTypeEnum_invalid = 0,
	lineTypeEnum_dash    = 1,
	lineTypeEnum_solid   = 2,
	lineTypeEnum_t_eMax
}lineTypeEnum_t;

typedef struct NEWCO_STRUCTURE_PACK
{
    GLfloat x;
    GLfloat y;
    GLfloat z;    
}point3DFloat_t;

typedef struct NEWCO_STRUCTURE_PACK
{
    point3DFloat_t one;
    point3DFloat_t two;
    point3DFloat_t three;
    point3DFloat_t four;
}quadShapeFloat_t;

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
	int number;  //how many point it has
	point3DFloat_t position[MAX_BUFFER_DEPTH_DRAW_LINE_POINT];
}lineInfo_t;

typedef struct NEWCO_STRUCTURE_PACK
{
    int number;
    quadShapeFloat_t buffer[MAX_BUFFER_DEPTH_3D_ENGINE];
}roadInfo_3D_Internal_t;

typedef struct NEWCO_STRUCTURE_PACK
{
    int number;
    signInfo_t buffer[MAX_BUFFER_DEPTH_3D_ENGINE];
}signInfo_3D_Internal_t;

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

typedef struct NEWCO_STRUCTURE_PACK
{
	int number;
	lineInfo_t line[MAX_BUFFER_NUMBER_DRAW_LINE];
}lineInfo_3D_Internal_t;

class OPENGL_3D_ENGNIE
{
public:
    OPENGL_3D_ENGNIE();
    ~OPENGL_3D_ENGNIE();
    
    //this function to add road inforation to sample 3D engine, it will copy all the road quadShape structure to 3D engine's backend internal buffer.
    GLboolean AddRoadInfo(int number,           //how many quadShapes we need to draw
                quadShapeFloat_t *buffer);   //this buffer can been delete after call this function
                
    //this function to add Sign inforation to sample 3D engine, it will copy the sign information to 3D engine's backend internal buffer.
    GLboolean AddSignInfo(int number, signInfo_t* buffer);  //add road side Sign inforamtion 

	//this function to add one line information to sample 3D engine, one time add one line.
	GLboolean AddOneLineInfo(int number,lineTypeEnum_t type, point3DFloat_t* buffer);

    //change the backend buffer to front buffer to draw, it means add roadInfomation and Sign information finished.
    GLboolean Swap3DBuffers(void);
    
    //change the eye look at backend buffer to front buffer to draw.
    GLboolean SwapEyeBuffer(void);

	//change the draw char backend buffer to front buffer to draw
	GLboolean SwapCharBuffer(void);
    
    //we don't set the head direction, default head direction is (0,1,0) ,  it will add the eyeBuffer to 3D engine's backend internal buffer.
    GLboolean setEyeLookat(int number,           //how many frames will draw based on the following information, this function is based one road inforamtion coordinates
                 eyeLookAt_t* buffer);
    
	//set a drawed character information,only support english charater, when finished add character information, pls call SwapCharBuffer() to show 
	GLboolean setDrawChar(drawCharInfo_t *buffer);

    void setEyeOnce(eyeLookAt_t eye);
    void DrawFrontBuffer();
    void DrawAll();
    //load 24 bit bmp picture as type, the input bmp must be 256x256, after loading, signInfo_t's sign field must match to 
	// this function's type value. 
    GLboolean load_bmp24_texture(const char* file_name, int type);
    
private:
    void DrawRoad(quadShapeFloat_t road);
    void DrawSign(signInfo_t sign);
    void DrawPole(GLfloat height);
	void DrawCharDynamic(drawCharInfo_t input);
	void DrawChar(drawCharInfo_t input);
	void DrawLine(lineInfo_t *line);
    
    int  power_of_two(int n);
    void CreateSignRoadSideShowList();
	void CreateCharaterShowList();
    
    int backBufIdx;         //backend buffer index
    int eyeBackBufIdx;      //eye backend buffer index
	int charBackBufIdx;     //draw charater backend buffer index
    roadInfo_3D_Internal_t RoadBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
    signInfo_3D_Internal_t signBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
    eyeInfo_3D_Internal_t  eyeBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
	lineInfo_3D_Internal_t lineBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];

	drawCharInfo_3D_Internal_t charBuffer[MAX_BUFFER_NUMBER_3D_ENGINE];
    GLuint                 texturelist[MAX_BUFFER_DEPTH_2D_TXETURE];
	
    
    int SignShowList_roadSide;
	int CharShowList;
	HANDLE hMutex;
};



            





             
             

