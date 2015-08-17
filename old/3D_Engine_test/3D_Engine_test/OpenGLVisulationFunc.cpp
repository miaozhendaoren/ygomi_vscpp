/*******************************************************************************
*                           Ygomi Confidential
*                  Copyright (c) Ygomi, LLC. 1995-2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  OpenGLVisulationFunc.cpp
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

#include "OpenGLVisulationFunc.h"

#define ColoredVertex(c, v) { glColor3fv(c); glVertex3fv(v);}

OPENGL_3D_ENGNIE::OPENGL_3D_ENGNIE()
{
    backBufIdx = 0;
    eyeBackBufIdx = 0;
	charBackBufIdx = 0;
    RoadBuffer[backBufIdx].number = 0;
    signBuffer[backBufIdx].number = 0;
	lineBuffer[backBufIdx].number = 0;

	charBuffer[charBackBufIdx].number = 0;
    eyeBuffer[eyeBackBufIdx].number  = 0;
	eyeBuffer[eyeBackBufIdx].index  = 0;
    
	glEnable(GL_TEXTURE_2D);
    glEnable(GL_DEPTH_TEST);

    
    CreateSignRoadSideShowList();
	CreateCharaterShowList();
    
    memset(texturelist,0,sizeof(GLuint)*MAX_BUFFER_DEPTH_2D_TXETURE);
	
	hMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(hMutex);
}

OPENGL_3D_ENGNIE::~OPENGL_3D_ENGNIE()
{
    int i = 0;
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
}

//copy the RoadInformation into backend buffer
GLboolean OPENGL_3D_ENGNIE::AddRoadInfo(
        int number,
        quadShapeFloat_t *buffer
)
{
    //check if the road information is larger than internal buffer
    if((RoadBuffer[backBufIdx].number+number) > MAX_BUFFER_DEPTH_3D_ENGINE)
    {
        return GL_FALSE;
    }
    
    //copy the content of quadShapeFloat_t buffer
    memcpy((void*)&RoadBuffer[backBufIdx].buffer[RoadBuffer[backBufIdx].number], (void*)buffer,number*sizeof(quadShapeFloat_t));
    
    //update the number
    RoadBuffer[backBufIdx].number += number;
    return GL_TRUE;
}

//copy the Sign information 
GLboolean OPENGL_3D_ENGNIE::AddSignInfo(
        int number, 
        signInfo_t* buffer)
{
    //check if the sign info is larger then internal buffer
    if((signBuffer[backBufIdx].number+number) > MAX_BUFFER_DEPTH_3D_ENGINE)
    {
        return GL_FALSE;
    }
    
    //copy the content of signInfo_t information
    memcpy((void*)&signBuffer[backBufIdx].buffer[signBuffer[backBufIdx].number], (void*)buffer,number*sizeof(signInfo_t));
    
    //update the number
    signBuffer[backBufIdx].number += number;
    return GL_TRUE;
}

GLboolean OPENGL_3D_ENGNIE::AddOneLineInfo(int number,lineTypeEnum_t type, point3DFloat_t* buffer)
{
	if((lineBuffer[backBufIdx].number+1) > MAX_BUFFER_NUMBER_DRAW_LINE)
    {
        return GL_FALSE;
    }

	//copy the content of signInfo_t information
    memcpy((void*)lineBuffer[backBufIdx].line[lineBuffer[backBufIdx].number].position, (void*)buffer,number*sizeof(point3DFloat_t));
    
    //update the number
	lineBuffer[backBufIdx].line[lineBuffer[backBufIdx].number].number = number;
	lineBuffer[backBufIdx].line[lineBuffer[backBufIdx].number].type  = type;

    lineBuffer[backBufIdx].number += 1;
    return GL_TRUE;
}

//
GLboolean OPENGL_3D_ENGNIE::setEyeLookat(int number,
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

GLboolean OPENGL_3D_ENGNIE::setDrawChar(drawCharInfo_t *buffer)
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
GLboolean OPENGL_3D_ENGNIE::Swap3DBuffers(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
    backBufIdx ^= 1;
    RoadBuffer[backBufIdx].number = 0;
    signBuffer[backBufIdx].number = 0;
	charBuffer[backBufIdx].number = 0;
	ReleaseMutex(hMutex);
    return GL_TRUE;
}

GLboolean OPENGL_3D_ENGNIE::SwapEyeBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE); 
    eyeBackBufIdx ^= 1;
    eyeBuffer[eyeBackBufIdx].number = 0;
    eyeBuffer[eyeBackBufIdx].index = 0;
	ReleaseMutex(hMutex);
    return GL_TRUE;
}

GLboolean OPENGL_3D_ENGNIE::SwapCharBuffer(void)
{
	WaitForSingleObject(hMutex,INFINITE);
	charBackBufIdx ^= 1;
	charBuffer[charBackBufIdx].number = 0;
	ReleaseMutex(hMutex);
    return GL_TRUE;
}
void OPENGL_3D_ENGNIE::setEyeOnce(eyeLookAt_t eye)
{
    glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(75,1,1,500);
	glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    gluLookAt(eye.eyePosition.x,eye.eyePosition.y,eye.eyePosition.z,eye.lookatPosition.x,eye.lookatPosition.y,eye.lookatPosition.z, 0, 1, 0);
    
}

void OPENGL_3D_ENGNIE::DrawFrontBuffer()
{
    int roadIdx,signIdx,charIdx,lineIdx;
    int frontBufIdx = backBufIdx^1;
	int charFrontBufIdx = charBackBufIdx^1;
    //draw all the road
    glPushMatrix();
    for(roadIdx = 0; roadIdx < RoadBuffer[frontBufIdx].number; roadIdx++)
    {
        DrawRoad(RoadBuffer[frontBufIdx].buffer[roadIdx]);
    }
    glPopMatrix();
   
    //draw all the sign
    for(signIdx = 0; signIdx < signBuffer[frontBufIdx].number; signIdx++)
    {
        DrawSign(signBuffer[frontBufIdx].buffer[signIdx]);
    }

	for(lineIdx = 0; lineIdx < lineBuffer[frontBufIdx].number; lineIdx++)
	{
		DrawLine(&lineBuffer[frontBufIdx].line[lineIdx]);
	}

	//draw all the charater 
	for(charIdx = 0; charIdx < charBuffer[charFrontBufIdx].number; charIdx++)
	{
		DrawChar(charBuffer[charFrontBufIdx].buffer[charIdx]);
	}
    
}

void OPENGL_3D_ENGNIE::DrawAll()
{
	WaitForSingleObject(hMutex,INFINITE); 
	{
		int frontBufIdx = eyeBackBufIdx^1;
		printf("%d",frontBufIdx);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
		//update eyeBuffer
		if(eyeBuffer[frontBufIdx].number != 0)
		{
			printf("index = %d\n",eyeBuffer[frontBufIdx].index);
		setEyeOnce(eyeBuffer[frontBufIdx].buffer[eyeBuffer[frontBufIdx].index]);
		}
    
		if(eyeBuffer[frontBufIdx].index < (eyeBuffer[frontBufIdx].number-1))
		{
			eyeBuffer[frontBufIdx].index += 1;
		}

		DrawFrontBuffer();

		glutSwapBuffers();
	}
	ReleaseMutex(hMutex);
    
}

void OPENGL_3D_ENGNIE::DrawRoad(quadShapeFloat_t road)
{
    //set raod's color
    //GLfloat ColorGray[] = {0.4f, 0.4f, 0.4f};
    glColor3f(0.4f,0.4f,0.4f);
    
    glBegin(GL_QUADS);
    glVertex3f(road.one.x, road.one.y, road.one.z);
    glVertex3f(road.two.x, road.two.y, road.two.z);
    glVertex3f(road.three.x, road.three.y, road.three.z);
    glVertex3f(road.four.x, road.four.y, road.four.z);
    glEnd();
}

void OPENGL_3D_ENGNIE::DrawSign(signInfo_t sign)
{
    GLint last_texture_ID;
    if((int)(sign.type) > MAX_BUFFER_DEPTH_2D_TXETURE)
    {
        return;
    }
    
    glPushMatrix();
    
    //draw the sign on the road side
    glTranslatef(sign.position.x, 0, sign.position.z);
	glRotatef(sign.rotAngle,0, 1, 0);
    DrawPole(sign.position.y - 0.5f);
    
    glTranslatef(0,sign.position.y,0);
    
    glGetIntegerv(GL_TEXTURE_BINDING_2D, &last_texture_ID);
    
    glBindTexture(GL_TEXTURE_2D, texturelist[sign.type]);
    glCallList(SignShowList_roadSide);
    glBindTexture(GL_TEXTURE_2D, last_texture_ID);
    
    glPopMatrix();
}

void OPENGL_3D_ENGNIE::DrawPole(GLfloat height)
{
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
      0,2,3,1,       //背面
      0,4,6,2,       //左侧面
      0,1,5,4,       //下面
      4,5,7,6,       //正面
      1,3,7,5,       //右侧面
      2,6,7,3,      //上面
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
}

void OPENGL_3D_ENGNIE::DrawLine(lineInfo_t *line)
{
	int index = 1;
	glPushMatrix();
	
	//虚线
	if(lineTypeEnum_dash == line->type)
	{
	    glEnable(GL_LINE_STIPPLE);
	    glLineStipple(10,0x00FF);
	
		glLineWidth(5.0f);

		glBegin(GL_LINE_STRIP);
		//set the color of line
		glColor3f(1.0f,0.95f,0.9f);  //white
		for(index = 0; index < line->number; index++)
		{
			glVertex3f(line->position[index].x, line->position[index].y, line->position[index].z);
		}
		glEnd();
	
		glDisable(GL_LINE_STIPPLE);
	}else if(lineTypeEnum_solid == line->type)
	{
		glLineWidth(5.0f);

		glBegin(GL_LINE_STRIP);
		//set the color of line
		glColor3f(1.0f,0.95f,0.9f);  //white
		for(index = 0; index < line->number; index++)
		{
			glVertex3f(line->position[index].x, line->position[index].y, line->position[index].z);
		}
		glEnd();
	}

	glPopMatrix();
}
void OPENGL_3D_ENGNIE::DrawCharDynamic(drawCharInfo_t input)
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

void OPENGL_3D_ENGNIE::DrawChar(drawCharInfo_t input)
{
	char* str = input.drawChar;
	glPushMatrix();
	
	glColor3f(1.0f, 1.0f, 0.0f);
	glRasterPos3f(input.position.x,input.position.y,input.position.z);
	//glRasterPos2f(input.position.x,input.position.y);

	for(;*str!='\0';++str)
      glCallList(CharShowList + *str);
	glPopMatrix();
}

void OPENGL_3D_ENGNIE::CreateSignRoadSideShowList()
{
    GLfloat
         Point1[] = {-0.5f, 0.5f, 0.01f},
         Point2[] = {0.5f,  0.5f, 0.01f},
         Point3[] = {0.5f, -0.5f, 0.01f},
         Point4[] = {-0.5f,-0.5f, 0.01f},
         Point5[] = {-0.5f, 0.5f, -0.01f},
         Point6[] = {0.5f,  0.5f, -0.01f},
         Point7[] = {0.5f, -0.5f, -0.01f},
         Point8[] = {-0.5f,-0.5f, -0.01f};
     GLfloat
         ColorR[] = {1, 0, 0,0.5},
         ColorY[] = {1, 1, 0,0.5};

	 SignShowList_roadSide = glGenLists(1);

     glNewList(SignShowList_roadSide, GL_COMPILE);
     glBegin(GL_QUADS);
     // 正面贴图
     glTexCoord2f(0.0f, 1.0f);
     ColoredVertex(ColorR, Point1);
     glTexCoord2f(1.0f, 1.0f);
     ColoredVertex(ColorR, Point2);
     glTexCoord2f(1.0f, 0.0f);
     ColoredVertex(ColorR, Point3);
     glTexCoord2f(0.0f, 0.0f);
     ColoredVertex(ColorR, Point4);

     // 反面
     ColoredVertex(ColorY, Point5);
     ColoredVertex(ColorY, Point6);
     ColoredVertex(ColorY, Point7);
     ColoredVertex(ColorY, Point8);

     // 侧面
     ColoredVertex(ColorR, Point1);
     ColoredVertex(ColorR, Point4);
     ColoredVertex(ColorY, Point8);
     ColoredVertex(ColorY, Point5);

     ColoredVertex(ColorR, Point1);
     ColoredVertex(ColorR, Point2);
     ColoredVertex(ColorY, Point6);
     ColoredVertex(ColorY, Point5);

     ColoredVertex(ColorR, Point2);
     ColoredVertex(ColorR, Point3);
     ColoredVertex(ColorY, Point7);
     ColoredVertex(ColorY, Point6);

     ColoredVertex(ColorR, Point3);
     ColoredVertex(ColorR, Point4);
     ColoredVertex(ColorY, Point8);
     ColoredVertex(ColorY, Point7);

     glEnd();
     glEndList();
}

void OPENGL_3D_ENGNIE::CreateCharaterShowList()
{
	//set the charater size and type
	HFONT hFont = CreateFontA(24, 0, 0, 0, FW_MEDIUM, 0, 0, 0,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS,
        DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, "Comic Sans MS");
	HFONT hOldFont = (HFONT)SelectObject(wglGetCurrentDC(), hFont);
	DeleteObject(hOldFont);

	CharShowList = glGenLists(128);
	wglUseFontBitmaps(wglGetCurrentDC(), 0, 128, CharShowList);
}

int OPENGL_3D_ENGNIE::power_of_two(int n)
{
    if( n <= 0 )
        return 0;
    return (n & (n-1)) == 0;
}

//
GLboolean OPENGL_3D_ENGNIE::load_bmp24_texture(const char* file_name, int type)
{
    GLint width, height, total_bytes;
    GLubyte* pixels = 0;
    GLuint last_texture_ID, texture_ID = 0;

     // 打开文件，如果失败，返回
    FILE* pFile = fopen(file_name, "rb");
    if( pFile == 0 )
        return GL_FALSE;

    if(type > MAX_BUFFER_DEPTH_2D_TXETURE)
    {
        return GL_FALSE;
    }
    // 读取文件中图象的宽度和高度
    fseek(pFile, 0x0012, SEEK_SET);
    fread(&width, 4, 1, pFile);
    fread(&height, 4, 1, pFile);
    fseek(pFile, BMP_Header_Length, SEEK_SET);

     // 计算每行像素所占字节数，并根据此数据计算总像素字节数
     {
         GLint line_bytes = width * 3;
        while( line_bytes % 4 != 0 )
             ++line_bytes;
         total_bytes = line_bytes * height;
     }

     // 根据总像素字节数分配内存
     pixels = (GLubyte*)malloc(total_bytes);
    if( pixels == 0 )
     {
        fclose(pFile);
        return GL_FALSE;
     }

     // 读取像素数据
    if( fread(pixels, total_bytes, 1, pFile) <= 0 )
     {
        free(pixels);
        fclose(pFile);
        return GL_FALSE;
     }

     // 在旧版本的OpenGL中
     // 如果图象的宽度和高度不是的整数次方，则需要进行缩放
     // 这里并没有检查OpenGL版本，出于对版本兼容性的考虑，按旧版本处理
     // 另外，无论是旧版本还是新版本，
     // 当图象的宽度和高度超过当前OpenGL实现所支持的最大值时，也要进行缩放
     {
         GLint max;
         glGetIntegerv(GL_MAX_TEXTURE_SIZE, &max);
        if( !power_of_two(width)
          || !power_of_two(height)
          || width > max
          || height > max )
         {
            const GLint new_width = 256;
            const GLint new_height = 256; // 规定缩放后新的大小为边长的正方形
             GLint new_line_bytes, new_total_bytes;
             GLubyte* new_pixels = 0;

             // 计算每行需要的字节数和总字节数
             new_line_bytes = new_width * 3;
            while( new_line_bytes % 4 != 0 )
                 ++new_line_bytes;
             new_total_bytes = new_line_bytes * new_height;

             // 分配内存
             new_pixels = (GLubyte*)malloc(new_total_bytes);
            if( new_pixels == 0 )
             {
                free(pixels);
                fclose(pFile);
                return GL_FALSE;
             }

             // 进行像素缩放
             gluScaleImage(GL_RGB,
                 width, height, GL_UNSIGNED_BYTE, pixels,
                 new_width, new_height, GL_UNSIGNED_BYTE, new_pixels);

             // 释放原来的像素数据，把pixels指向新的像素数据，并重新设置width和height
            free(pixels);
             pixels = new_pixels;
             width = new_width;
             height = new_height;
         }
     }

     // 分配一个新的纹理编号
     glGenTextures(1, &texture_ID);
    if( texture_ID == 0 )
     {
        free(pixels);
        fclose(pFile);
        return GL_FALSE;
     }

     // 绑定新的纹理，载入纹理并设置纹理参数
     // 在绑定前，先获得原来绑定的纹理编号，以便在最后进行恢复
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

     // 之前为pixels分配的内存可在使用glTexImage2D以后释放
     // 因为此时像素数据已经被OpenGL另行保存了一份（可能被保存到专门的图形硬件中）
    free(pixels);
    
    texturelist[type] = texture_ID;
    return GL_TRUE;
}



