#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include "ThreadBase.h"

class CThreadBase;

class DCICOMMONEXT_MODULE_EXPIMP CTask
{
public:
	enum WaitEnums
	{
		NotWait	= 0,
		Wait    = 1
	};

protected:

	// �����ڵ�ǰ�̵߳�������
	TaskNumber m_Number;

	CThreadBase  *m_pProducer;	// ���õ�����Ĵ����ߣ�������ʱ�����ͷ�����Դ
	CThreadBase  *m_pCustomer;	// ��������������ߣ��������ִ���ߣ�������ʱ�����ͷ�����Դ
	void		 *m_pContext;	// ���õ������Ļ�����������ʱ������������Դ

	bool m_bNeedResponse;
	bool m_bUseForResponse;		// ָʾ��ǰ����ֻ����Ϊ֮ǰͬһ�������ŵ�ȡ������
	

public:
	Ext::CPP::Event Command;	// �����ִ�к���
	Panic State;				// ��¼�����״̬��ֻ����Ӧ������Ч��������Ӧ����ʱ����ǰ����� m_Panic �ᴫ�ݵ���Ӧ����Ӧ����� m_Panic

private:
	CTask();
	~CTask();

public:
	static CTask* CreateTask(CThreadBase *pProducer, CThreadBase *pCustomer);
	static void ReleaseTask(CTask *&pTask);
	CTask* CreateResponseTask();

	// �����ڲ�����
	virtual void Do();
	void SetTaskNumber(TaskNumber num);

	CThreadBase* GetProducer();
	CThreadBase* GetCustomer();

	bool IsUseForResponse();
	void SetUseForResponse(bool response);
	bool IsNeedResponse();
	TaskNumber GetTaskNumber();
	std::wstring ToString();

	// ����ǰ������䵽 Owner ���߳��У�����ִ�����Ժ��Զ�����ǰ Task ʵ������Դ
	TaskNumber Assign(void *pContext, bool needResponse);
	static Panic Assign(WaitEnums wait, Panic state, void *pContext, EventHandle handle, CThreadBase *pFrom, CThreadBase *pTo);

	// ָ������ת�������ã�CAdoRow *pRow = pTask->GetAttach<CAdoRow*>();	// pTask ��CTask���͵�ָ�룬Attach ָ��ָ��
	template<typename T>
	T GetContext()
	{
		T pointer = static_cast<T>(m_pContext);
		return pointer;
	}
};