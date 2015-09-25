/*******************************************************************************
*                           Continental Confidential
*                  Copyright (c) Continental, LLC. 2015
*
*      This software is furnished under license and may be used or
*      copied only in accordance with the terms of such license.
*******************************************************************************
* @file  main.cpp
* @brief main function
*
* Change Log:
*      Date                Who             What
*      2014/12/25         Xin Shao        Create
*******************************************************************************
*/

#include <stdio.h>
#include <Windows.h>    //HANDLE header file
#include <process.h>    //_beginthread header file
#include <gl/glut.h>
#include <math.h>

#include "AppInitCommon.h"
#include "Thread_VisualizePreProc.h"
#include "Thread_RecvSensors.h"
#include "Thread_DBUpdate.h"
#include "Thread_DiffDetRpt.h"
#include "Thread_ImageSensorCollector.h"

#include "Visualization.h"
#include "VisualizeControl.h"


#pragma comment(lib, "ws2_32.lib") 

int main(int argc, char* argv[])
{
	//do some system initialation
	appInitEvents();
    databaseInit();
    //config file parser
	if(!configParameterInit())
	{
		cout<<"parse config file  fail!!!"<<endl;
		return -1;
	}	
	
	int status = detectorInit();
    if(status != 0)
    {
        return -1;
    }

	//getVehicleID(&g_VehicleID);
	startSocket();
	sendDatabaseVersion();
	//initialize the glut
	 glutInit(&argc, argv);
     glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
     glutInitWindowPosition(200, 200);
     glutInitWindowSize(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT);
     glutCreateWindow("Vehicle View");

	 Init3DEngine();

	//create each thread, there are three threads.
	//now use _beginthreadex to create thread for more flexible
	// _beginthreadex has more parameter to control the thread
	HANDLE threadHandle[5];

	threadHandle[0] = (HANDLE) _beginthreadex(0,0,&Thread_DBUpdate,0,0,0);
	threadHandle[1] = (HANDLE) _beginthreadex(0,0,&Thread_DiffDetRpt,0,0,0);
	threadHandle[2] = (HANDLE) _beginthreadex(0,0,&Thread_VisualizePreProc,0,0,0);
	threadHandle[3] = (HANDLE) _beginthreadex(0,0,&Thread_ReconnectSocket,0,0,0);
	threadHandle[4] = (HANDLE) _beginthreadex(0,0,&Thread_ImageSensorCollector,0,0,0);
	
	//set thread priority
	SetThreadPriority(threadHandle[0],THREAD_PRIORITY_NORMAL);
	SetThreadPriority(threadHandle[1],THREAD_PRIORITY_LOWEST);
	SetThreadPriority(threadHandle[2],THREAD_PRIORITY_NORMAL);
	SetThreadPriority(threadHandle[3],THREAD_PRIORITY_NORMAL);
	SetThreadPriority(threadHandle[4],THREAD_PRIORITY_ABOVE_NORMAL);

	//SetThreadAffinityMask(threadHandle[0],0x00000001);
	//SetThreadAffinityMask(threadHandle[1],0x00000002);
	//SetThreadAffinityMask(threadHandle[2],0x00000003);
	//SetThreadAffinityMask(threadHandle[3],0x00000004);

	glutDisplayFunc(&myDisplay);

	glutTimerFunc((unsigned int)(1000/FRAME_NUM_PER_SECOND),&myTimer,1);
	glutKeyboardFunc(&keyboardFunc);
	glutSpecialFunc(&specialKeyFunc);
	glutReshapeFunc(&reshapeWindow);
	glutMouseFunc(&mouseButtonFunc);
	glutMotionFunc(&mouseMoveFunc);
	glutMainLoop();
	

    WaitForMultipleObjects(5, threadHandle, true, INFINITE);

	CloseHandle(threadHandle[0]);
	CloseHandle(threadHandle[1]);
	CloseHandle(threadHandle[2]);
	CloseHandle(threadHandle[3]);
	CloseHandle(threadHandle[4]);

	delete3DEngine();
	WSACleanup();
	return 0;
}