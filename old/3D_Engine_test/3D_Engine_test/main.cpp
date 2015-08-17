#include <stdio.h>
#include <gl/glut.h>
#include "OpenGLVisulationFunc.h"

//multi-thread windows supported header file
#include <Windows.h>
#include <process.h>     //_beginthread header file
#include <math.h>        

OPENGL_3D_ENGNIE *engine3DPtr;
const GLfloat Pi = 3.1415926536f;

void myTimer1(int value)
{
#if 1
	int idx = 0;
	GLfloat speed = 0.5;
	glutTimerFunc(1000,&myTimer1,2);

	eyeLookAt_t  eyeInfo1[100];
	
	eyeInfo1[0].eyePosition.x = -2.5;
	eyeInfo1[0].eyePosition.y = 2;
	eyeInfo1[0].eyePosition.z = 1;
	eyeInfo1[0].lookatPosition.x = eyeInfo1[0].eyePosition.x;
	eyeInfo1[0].lookatPosition.y = 1.7;
	eyeInfo1[0].lookatPosition.z = eyeInfo1[0].eyePosition.z -1;
	for(idx = 1; idx < 10; idx++)
	{
		eyeInfo1[idx].eyePosition.x = eyeInfo1[idx-1].eyePosition.x;
		eyeInfo1[idx].eyePosition.y = eyeInfo1[idx-1].eyePosition.y;
		eyeInfo1[idx].eyePosition.z = eyeInfo1[idx-1].eyePosition.z - speed;
		eyeInfo1[idx].lookatPosition.x = eyeInfo1[idx-1].lookatPosition.x;
		eyeInfo1[idx].lookatPosition.y = eyeInfo1[idx-1].lookatPosition.y;
		eyeInfo1[idx].lookatPosition.z = eyeInfo1[idx-1].lookatPosition.z - speed;
	}

	engine3DPtr->setEyeLookat(10,eyeInfo1);
	engine3DPtr->SwapEyeBuffer();
#endif
}



//this thread is used to set the input to 3d engine
void mythread1(void *data)
{
	quadShapeFloat_t roadInfo[100];
	eyeLookAt_t  eyeInfo1[100];
	//eyeLookAt_t  eyeInfo2[100];
	signInfo_t   signInfo[3];
	drawCharInfo_t charInfo[2];
	
	point3DFloat_t line1[4];
	point3DFloat_t line2[4];
    point3DFloat_t line3[4];

	//load all the texture file(limited to 1024)
	//engine3DPtr->load_bmp24_texture("stop-sign-model.bmp",1);  //this means sign type 1 is this texture

	//add all the road and sign information to 3D engine
	roadInfo[0].one.x = -5.0f;
	roadInfo[0].one.y = 0;
	roadInfo[0].one.z = 0;
    roadInfo[0].two.x = -5.0f;
	roadInfo[0].two.y = 0;
	roadInfo[0].two.z = -20;
	roadInfo[0].three.x = 5.0f;
	roadInfo[0].three.y = 0;
	roadInfo[0].three.z = -20;
	roadInfo[0].four.x = 5.0f;
	roadInfo[0].four.y = 0;
	roadInfo[0].four.z = 0;

	roadInfo[1].one.x = -5.0f;
	roadInfo[1].one.y = 0;
	roadInfo[1].one.z = -20;
    roadInfo[1].two.x = 20.0f;
	roadInfo[1].two.y = 0;
	roadInfo[1].two.z = -35;
	roadInfo[1].three.x = 35;
	roadInfo[1].three.y = 0;
	roadInfo[1].three.z = -35;
	roadInfo[1].four.x = 5.0f;
	roadInfo[1].four.y = 0;
	roadInfo[1].four.z = -20;

	//sign info
	signInfo[0].type = 2;
	signInfo[0].rotAngle = 0;
	signInfo[0].position.x = -5.0f;
	signInfo[0].position.y = 2.0f;
	signInfo[0].position.z = -15.0f;
	signInfo[0].attribute = 0;    //reserved, used for extention

	signInfo[1].type = 1;
	signInfo[1].rotAngle = -180*atan2(-(roadInfo[1].two.z - roadInfo[0].two.z),(roadInfo[1].two.x - roadInfo[0].two.x))/Pi;
	signInfo[1].position.x = 7.0f;
	signInfo[1].position.y = 2.0f;
	signInfo[1].position.z = -25.0f;

	charInfo[0].position.x = 0.0f;
	charInfo[0].position.y = 5.0f;
	charInfo[0].position.z = -20.0f;
	memcpy((void*)charInfo[0].drawChar,(void*)"front 25M, Stop Sign",21);

	charInfo[1].position.x = 2.0f;
	charInfo[1].position.y = 3.0f;
	charInfo[1].position.z = -25.0f;
	memcpy((void*)charInfo[1].drawChar,(void*)"front 25M, Stop Sign",21);

	eyeInfo1[0].eyePosition.x = -2.5;
    eyeInfo1[0].eyePosition.y = 2;
	eyeInfo1[0].eyePosition.z = 1;
	eyeInfo1[0].lookatPosition.x = eyeInfo1[0].eyePosition.x;
    eyeInfo1[0].lookatPosition.y = 1.7;
	eyeInfo1[0].lookatPosition.z = eyeInfo1[0].eyePosition.z -1;

	eyeInfo1[1].eyePosition.x = -2.5;
    eyeInfo1[1].eyePosition.y = 2;
	eyeInfo1[1].eyePosition.z = 0;
	eyeInfo1[1].lookatPosition.x = eyeInfo1[1].eyePosition.x;
    eyeInfo1[1].lookatPosition.y = 1.7;
	eyeInfo1[1].lookatPosition.z = eyeInfo1[1].eyePosition.z -1;

    eyeInfo1[2].eyePosition.x = -2.5;
    eyeInfo1[2].eyePosition.y = 2;
	eyeInfo1[2].eyePosition.z = -1;
	eyeInfo1[2].lookatPosition.x = eyeInfo1[2].eyePosition.x;
    eyeInfo1[2].lookatPosition.y = 1.7;
	eyeInfo1[2].lookatPosition.z = eyeInfo1[2].eyePosition.z -1;

	line1[0].x = -5.0f;
	line1[0].y = 0.01f;
	line1[0].z = 0.0f;
	line1[1].x = -5.0f;
	line1[1].y = 0.01f;
	line1[1].z = -5.0f;
	line1[2].x = -5.0f;
	line1[2].y = 0.01f;
	line1[2].z = -10.0f;
	line1[3].x = -5.0f;
	line1[3].y = 0.01f;
	line1[3].z = -20.0f;

	line2[0].x = 5.0f;
	line2[0].y = 0.01f;
	line2[0].z = 0.0f;
	line2[1].x = 5.0f;
	line2[1].y = 0.01f;
	line2[1].z = -5.0f;
	line2[2].x = 5.0f;
	line2[2].y = 0.01f;
	line2[2].z = -10.0f;
	line2[3].x = 5.0f;
	line2[3].y = 0.01f;
	line2[3].z = -20.0f;

	line3[0].x = 0.0f;
	line3[0].y = 0.01f;
	line3[0].z = 0.0f;
	line3[1].x = 0.0f;
	line3[1].y = 0.01f;
	line3[1].z = -5.0f;
	line3[2].x = 0.0f;
	line3[2].y = 0.01f;
	line3[2].z = -10.0f;
	line3[3].x = 0.0f;
	line3[3].y = 0.01f;
	line3[3].z = -20.0f;


	engine3DPtr->AddRoadInfo(2, roadInfo);
	engine3DPtr->AddSignInfo(2,signInfo);
	engine3DPtr->AddOneLineInfo(4,lineTypeEnum_solid,line1);
	engine3DPtr->AddOneLineInfo(4,lineTypeEnum_solid,line2);
	engine3DPtr->AddOneLineInfo(4,lineTypeEnum_dash,line3);
	engine3DPtr->setEyeLookat(3,eyeInfo1);
	engine3DPtr->setDrawChar(&charInfo[0]);
	//engine3DPtr->setDrawChar(&charInfo[1]);
	engine3DPtr->Swap3DBuffers();
	engine3DPtr->SwapEyeBuffer();
	engine3DPtr->SwapCharBuffer();

	glutTimerFunc(1000,&myTimer1,2);
	while(1)
	{

	}
}


void myDisplay(void)
{
	engine3DPtr->DrawAll();
}

void myTimer(int value)
{
	if(1 == value)
	{
		glutTimerFunc(100,&myTimer,1);
		myDisplay();
	}
}
int main(int argc, char* argv[])
{
	 const char* version;
	 HANDLE myhandler1;
     glutInit(&argc, argv);
     glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
     glutInitWindowPosition(200, 200);
     glutInitWindowSize(600, 400);
     glutCreateWindow("3D Engine test");

	 //检查自己系统中的OPENGL版本
	 version = (const char*)glGetString(GL_VERSION);
	 printf("OpenGL 版本：%s\n",version);
	 printf("显卡名字：%s\n",glGetString(GL_RENDERER));
	 printf("支持的所有扩展：%s\n",glGetString(GL_EXTENSIONS));

	 engine3DPtr = new OPENGL_3D_ENGNIE();
	 engine3DPtr->load_bmp24_texture("traffic-light.bmp",1);
	 engine3DPtr->load_bmp24_texture("turn-right.bmp",2);

	 myhandler1 = (HANDLE)_beginthread(&mythread1,0,0);

	 glutDisplayFunc(&myDisplay);

	 glutTimerFunc(100,&myTimer,1);

	 glutMainLoop();

	 delete engine3DPtr;

}