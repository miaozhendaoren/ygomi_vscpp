
// InVehicle_Emulator.h : main header file for the PROJECT_NAME application
//

#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// CInVehicle_EmulatorApp:
// See InVehicle_Emulator.cpp for the implementation of this class
//

class CInVehicle_EmulatorApp : public CWinApp
{
public:
	CInVehicle_EmulatorApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CInVehicle_EmulatorApp theApp;