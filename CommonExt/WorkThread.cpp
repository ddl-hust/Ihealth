#include "stdafx.h"
#include "WorkThread.h"
#include "Task.h"

HANDLE CWorkThread::sm_hMutex = 0;
int CWorkThread::sm_WorkerCount = 0;
List<CWorkThread*> CWorkThread::sm_AvailableWorker;
List<CWorkThread*> CWorkThread::sm_BusyWorker;

CWorkThread::CWorkThread() : CThreadBase()
{
	m_bAutoReuse = true;
	sm_WorkerCount += 1;

	wchar_t name[256];
	wsprintf(name, _T("WorkThread_%d"), sm_WorkerCount);
	CThreadBase::m_Name = name;
	CThreadBase::InitialThread();
}

CWorkThread::~CWorkThread()
{
	CWorkThread::OnDispose();
}

CWorkThread* CWorkThread::GetWorker()
{
	// �״ε���ʱ�����г�ʼ��
	if (sm_hMutex == 0)
	{
		// ���� Mutex
		sm_hMutex = ::CreateMutex(
			NULL,                       // Ĭ�ϰ�ȫ����
			FALSE,                      // ��ʼ��Ϊδ��ӵ��
			NULL);                      // δ����
		if (sm_hMutex == NULL) 
		{
			printf("CreateMutex error: %d\n", GetLastError());
			return NULL;
		}
		sm_AvailableWorker.Reserve(10);
		sm_AvailableWorker.Clear();

		sm_BusyWorker.Reserve(10);
		sm_BusyWorker.Clear();
	}

	// �ȴ� Mutex
	DWORD dwWaitResult = ::WaitForSingleObject(sm_hMutex, INFINITE);

	CWorkThread *pWorkThread = NULL;
	if (dwWaitResult == WAIT_OBJECT_0)
	{
		if (sm_AvailableWorker.Count() > 0)
		{
			pWorkThread = sm_AvailableWorker.Last();
			sm_AvailableWorker.Remove();
		}
		else
		{
			pWorkThread = new CWorkThread();
		}
	}

	if (pWorkThread != NULL)
		AddWorker(sm_BusyWorker, pWorkThread);

	// �ͷ� Mutex
	if (!::ReleaseMutex(sm_hMutex))
	{
		printf("Release Mutex error: %d\n", GetLastError()); 
	}

	return pWorkThread;
}

void CWorkThread::Dispose()
{
	// �ȴ� Mutex
	DWORD dwWaitResult = ::WaitForSingleObject(sm_hMutex, INFINITE);

	for (int i = 0; i < sm_BusyWorker.Count(); ++i)
	{
		CWorkThread *pThread = sm_BusyWorker[i];
		delete pThread;
	}
	sm_BusyWorker.Clear();

	for (int i = 0; i < sm_AvailableWorker.Count(); ++i)
	{
		CWorkThread *pThread = sm_AvailableWorker[i];
		delete pThread;
	}
	sm_AvailableWorker.Clear();

	// �ͷ� Mutex
	if (!::ReleaseMutex(sm_hMutex))
	{
		printf("Release Mutex error: %d\n", GetLastError()); 
	}
}

void CWorkThread::Destroy()
{
	if (sm_hMutex != 0)
	{
		::CloseHandle(sm_hMutex);
		sm_hMutex = 0;
	}

	sm_WorkerCount = 0;
	sm_AvailableWorker.Clear();
	sm_BusyWorker.Clear();
}

void CWorkThread::OnFinishOneTask()
{
	if (m_bAutoReuse == false)
		return;

	this->Reuse();
}

bool CWorkThread::IsAutoReuse()
{
	return m_bAutoReuse;
}

void CWorkThread::SetAutoReuse(bool isAuto)
{
	m_bAutoReuse = isAuto;
}

void CWorkThread::Reuse()
{
	if (m_bReceiveQuitMessage == true)
		return;

	// �ȴ� Mutex
	DWORD dwWaitResult = ::WaitForSingleObject(sm_hMutex, INFINITE);

	AddWorker(sm_AvailableWorker, this);
	RemoveWorker(sm_BusyWorker, this);
	//DUITRACE(_T("Reuse work thread %s"), m_Name);

	// �ͷ� Mutex
	if (!::ReleaseMutex(sm_hMutex))
	{
		printf("Release Mutex error: %d\n", GetLastError()); 
	}
}

void CWorkThread::AddWorker(List<CWorkThread*> &list, CWorkThread *pWork)
{
	for (int i = 0; i < list.Count(); ++i)
	{
		CWorkThread *pThread = list[i];
		if (pThread->m_ThreadID == pWork->m_ThreadID)
			return;
	}
	list.Append(pWork);
}

void CWorkThread::RemoveWorker(List<CWorkThread*> &list, CWorkThread *pWork)
{
	for (int i = list.Count() - 1; i >= 0; --i)
	{
		CWorkThread *pThread = list[i];
		if (pThread->m_ThreadID == pWork->m_ThreadID)
		{
			list.Remove(i);
			return;
		}
	}
}