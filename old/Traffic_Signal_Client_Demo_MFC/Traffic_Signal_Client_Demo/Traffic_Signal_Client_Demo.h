
// Traffic_Signal_Client_Demo.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CTraffic_Signal_Client_DemoApp:
// See Traffic_Signal_Client_Demo.cpp for the implementation of this class
//

class CTraffic_Signal_Client_DemoApp : public CWinApp
{
public:
	CTraffic_Signal_Client_DemoApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CTraffic_Signal_Client_DemoApp theApp;