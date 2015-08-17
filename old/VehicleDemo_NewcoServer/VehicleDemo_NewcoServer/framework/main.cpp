/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  main.cpp
* @brief main function and create three threads.
*
* Change Log:
*      Date                Who             What
*      2015/01/13         Qin Shi         Create
*******************************************************************************
*/
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include <gl/glut.h>

#include "typeDefine.h"
#include "appInitCommon.h"
#include "Thread_Receive_Message.h"
#include "Thread_Master_DBAccess.h"
#include "Thread_Update_Message.h"
#include "Thread_VisualizePreProc.h"
#include "Visualization.h"
#include "VisualizeControl.h"

#pragma comment(lib, "ws2_32.lib") 
#define ON   1
#define OFF  0

int main(int argc, char* argv[])
{
	HANDLE threadHandle[4];
	appInitEvents();
	databaseInit();
	msgQueueInit();
	startSocket();

	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
    glutInitWindowPosition(200, 200);
    glutInitWindowSize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
    glutCreateWindow("Server View");

    Init3DEngine();

	threadHandle[0] = (HANDLE) _beginthreadex(0,0,&Thread_Receive_Message,0,0,0);
	threadHandle[1] = (HANDLE) _beginthreadex(0,0,&Thread_Master_DBAccess,0,0,0);
	threadHandle[2] = (HANDLE) _beginthreadex(0,0,&Thread_Update_Message,0,0,0);
	threadHandle[3] = (HANDLE) _beginthreadex(0,0,&Thread_VisualizePreProc,0,0,0);
		
	//set thread priority
	SetThreadPriority(threadHandle[0],THREAD_PRIORITY_ABOVE_NORMAL); //+1 priority
	SetThreadPriority(threadHandle[1],THREAD_PRIORITY_NORMAL); //+1 priority
	SetThreadPriority(threadHandle[2],THREAD_PRIORITY_ABOVE_NORMAL); //+1 priority
	SetThreadPriority(threadHandle[3],THREAD_PRIORITY_ABOVE_NORMAL); //+1 priority

	glutDisplayFunc(&myDisplay);
	glutTimerFunc((unsigned int)(1000/FRAME_NUM_PER_SECOND),&myTimer,1);
	glutKeyboardFunc(&keyboardFunc);
	glutSpecialFunc(&specialKeyFunc);
	glutReshapeFunc(&reshapeWindow);
	glutMouseFunc(&mouseButtonFunc);
	glutMotionFunc(&mouseMoveFunc);
	glutMainLoop();

	WaitForMultipleObjects(4, threadHandle, true, INFINITE);

	CloseHandle(threadHandle[0]);
	CloseHandle(threadHandle[1]);
	CloseHandle(threadHandle[2]);
	CloseHandle(threadHandle[3]);
	delete3DEngine();
	WSACleanup();
	return 0;
}