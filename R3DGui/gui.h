#ifndef __GUI__
#define __GUI__
// gui.h : main header file for the PROJECT_NAME application

#pragma once
#include "stdafx.h"
#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"		// main symbols


// See gui.cpp for the implementation of this class
/*! \class See gui.cpp for the implementation of this class
*   \author Martin Jurik
*/
class CguiApp : public CWinApp
{
public:
	CguiApp();

// Overrides
public:
	virtual BOOL InitInstance();

// Implementation

	DECLARE_MESSAGE_MAP()
};

extern CguiApp theApp;

#endif __GUI__