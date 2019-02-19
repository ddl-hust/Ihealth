#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include "ThreadBase.h"

#include "List.h"
using namespace Ext::Collection;

class DCICOMMONEXT_MODULE_EXPIMP CWorkThread : public CThreadBase
{
private:
	static HANDLE sm_hMutex;	// ͬ�����Work�߳�ͬʱ���� m_WorkerThreadPool �Ļ�����
	static int sm_WorkerCount;
	static List<CWorkThread*> sm_AvailableWorker;
	static List<CWorkThread*> sm_BusyWorker;

	bool m_bAutoReuse;	// �Զ����ã��������һ��������Զ��ؽ���ǰ�̻߳��յ��̳߳��С�Ĭ��Ϊ true

public:
	// ��ȡ��ǰ�̳߳���һ�����õ� Worker �߳�
	// ����ǰ���е� Worker ����æµ���򴴽����̷߳���
	static CWorkThread* GetWorker();

	static void Dispose();
	static void Destroy();

	bool IsAutoReuse();
	void SetAutoReuse(bool isAuto);

	// �ֶ��ؽ���ǰ�̻߳��յ��̳߳���
	void Reuse();

	// ��д����ִ����һ������󣬽���ǰ�̻߳��յ��̳߳���
	virtual void OnFinishOneTask();

private:
	CWorkThread();
	~CWorkThread();

	static void AddWorker(List<CWorkThread*> &list, CWorkThread *pWork);
	static void RemoveWorker(List<CWorkThread*> &list, CWorkThread *pWork);
};