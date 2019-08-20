#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include "ThreadBase.h"

class DCICOMMONEXT_MODULE_EXPIMP CUIThread : public CThreadBase
{
private:
	HWND m_Hwnd;

private:
	CUIThread(HWND hwnd);
	~CUIThread();

public:
	static Panic Create(HWND hwnd);
	static Panic Release(CUIThread *&pThread);

	virtual TaskNumber AssignTask(CTask *pTask);
	
	/// <summary>
	/// ������ UI �̵߳ȴ�����Ӧ�����񣬷�����潫�������״̬
	/// </summary>
	/// <author name="hps" date="2012.11.23   13:40"/>
	virtual Panic WaitResponse(TaskNumber num);
};