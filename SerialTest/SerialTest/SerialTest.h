
// SerialTest.h : PROJECT_NAME Ӧ�ó������ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CSerialTestApp: 
// �йش����ʵ�֣������ SerialTest.cpp
//

class CSerialTestApp : public CWinApp
{
public:
	CSerialTestApp();

// ��д
public:
	virtual BOOL InitInstance();

// ʵ��

	DECLARE_MESSAGE_MAP()
};

extern CSerialTestApp theApp;