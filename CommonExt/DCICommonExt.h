// DCICommonExt.h : DCICommonExt DLL ����ͷ�ļ�
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�ڰ������ļ�֮ǰ������stdafx.h�������� PCH �ļ�"
#endif

#include "resource.h"		// ������


// CDCICommonExtApp
// �йش���ʵ�ֵ���Ϣ������� DCICommonExt.cpp
//

class CDCICommonExtApp : public CWinApp
{
public:
	CDCICommonExtApp();

// ��д
public:
	virtual BOOL InitInstance();

	DECLARE_MESSAGE_MAP()
};
