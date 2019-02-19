#include "stdafx.h"
#include "ThreadBase.h"
#include "Task.h"

struct ThreadInfo
{
	CThreadBase *Threader;
};

void CThreadBase::ThreadFunc(LPVOID lpParam)
{
	ThreadInfo *pInfo = (ThreadInfo*)lpParam;
	CThreadBase *pThreader = pInfo->Threader;
	pThreader->RunMessageLoop(false, 0);
}

CThreadBase::CThreadBase()
{
	m_hMutex = ::CreateMutex(NULL, FALSE, NULL);
	m_TopTaskNumber = 100;	// [0, 100] ֮��������ű���
	m_bIsSuspend = false;
	m_bRefThread = false;
	m_bReceiveQuitMessage = false;

	pInfo = new ThreadInfo();
	pInfo->Threader = this;
}

CThreadBase::~CThreadBase()
{
	delete pInfo;
	pInfo = NULL;

	m_HThread = NULL;
	m_ThreadID = 0;

	::CloseHandle(m_hMutex);
	m_hMutex = 0;
}

void CThreadBase::OnDispose()
{
	if (m_bRefThread == false)
	{
		this->Resume();	// ��̨�߳��п��ܱ�������
		::PostThreadMessage(m_ThreadID, WM_THREAD_WANT_QUIT, NULL, NULL);
		::WaitForSingleObject(m_HThread, INFINITE);				// ���õ��̱߳���ȴ���̨�߳��˳������������ڴ�й©
		::CloseHandle(m_HThread);
	}
}

void CThreadBase::InitialThread()
{
	m_HThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CThreadBase::ThreadFunc, pInfo, 0, &m_ThreadID);
}

Panic CThreadBase::RunMessageLoop(bool answerTask, TaskNumber num)
{
	Panic panic;
	MSG msg = { 0 };
	while (::GetMessage(&msg, NULL, 0, 0))
	{
		if (msg.message == WM_THREAD_WANT_QUIT)
		{
			m_bReceiveQuitMessage = true;
			if (!answerTask)
				break;

			//DUITRACE(_T("%s Receive quit message"), m_Name);
			//DUITRACE(_T("%s Fore to answer the waiting response task %d"), m_Name, num);

			CTask *pTask = NULL;
			List<CTask*> oldTasks;
			MSG oldmsg = { 0 };
			while (::PeekMessage(&oldmsg, 0, 0, 0, PM_REMOVE))
			{
				if (oldmsg.message != WM_COMMUNICATE)
					continue;	// ���ܽ��յ�ϵͳ��������Ϣ

				CTask *pAnswerTask = (CTask*)oldmsg.lParam;
				if (pAnswerTask->GetTaskNumber() == num && pAnswerTask->IsUseForResponse())
				{
					pTask = pAnswerTask;
					break;
				}
				else
				{
					oldTasks.Append(pAnswerTask);
				}
			}
			if (pTask == NULL)
			{
				pTask = CTask::CreateTask(this, this);
				pTask->SetTaskNumber(num);
				pTask->SetUseForResponse(true);
				pTask->State.Set(WM_THREAD_WANT_QUIT, _T("Thread %s want to exit"), m_Name);
			}

			pTask->Assign(NULL, false);

			for (int i = 0; i < oldTasks.Count(); ++i)
				::PostThreadMessage(m_ThreadID, WM_COMMUNICATE, NULL, LPARAM(oldTasks[i]));

			// ���·����˳���Ϣ
			::PostThreadMessage(m_ThreadID, WM_THREAD_WANT_QUIT, NULL, NULL);
		}

		if (msg.message != WM_COMMUNICATE)
			continue;	// ���ܽ��յ�ϵͳ��������Ϣ

		CTask *pTask = (CTask*)msg.lParam;
		if (answerTask == true)
		{			
			if (pTask->GetTaskNumber() == num)	// �ȵ��˵�ǰ����ķ�������
			{
				panic = pTask->State;
				CTask::ReleaseTask(pTask);
				pTask = NULL;

				for (int i = 0; i < m_ResponseTask.Count(); ++i)
				{
					// ��ǰ���յ���Ӧ���������·��͵���ǰ�̵߳���Ϣ������
					CTask *pAnswerTask = m_ResponseTask[i];
					::PostThreadMessage(m_ThreadID, WM_COMMUNICATE, NULL, LPARAM(pAnswerTask));
				}
				m_ResponseTask.Clear();
				break;
			}

			if (pTask->IsUseForResponse() == true)
			{
				// �յ�ǰ����Ҫ������Ϣ�������Ӧ�𣬲���ֱ�Ӷ���
				m_ResponseTask.Append(pTask);
				continue;
			}
		}

		// ִ������
		this->ExecuteTask(pTask);
		pTask = NULL;
	}
	return panic;
}

Panic CThreadBase::WaitResponse(TaskNumber num)
{
	Panic panic = this->RunMessageLoop(true, num);
	return panic;
}

void CThreadBase::ExecuteTask(CTask *pTask)
{
	// ִ������
	pTask->Do();

	if (pTask->IsNeedResponse() == true)
	{
		CTask *pResponseTask = pTask->CreateResponseTask();
		//DUITRACE(_T("Response to task (%s)"), pResponseTask->ToString());
		pResponseTask->Assign(NULL, false);
	}

	CTask::ReleaseTask(pTask);
	this->OnFinishOneTask();
}

TaskNumber CThreadBase::AssignTask(CTask *pTask)
{
	std::wstring taskInfo = pTask->ToString();
	//DUITRACE(_T("ThreadBase assign task. (%s)"), taskInfo);

	TaskNumber num = pTask->GetTaskNumber();
	int count = 5;
	while (count > 0)
	{
		// ��̨�̵߳���Ϣ��������δ����������̷߳�����Ϣ�����ͻ�ʧ�ܡ����ִ�������ĸ��ʼ���
		// ���� PostThreadMessage �Ժ󣬵�ǰ�̲߳��ܶ� pTask ���κη���
		BOOL ok = ::PostThreadMessage(m_ThreadID, WM_COMMUNICATE, NULL, LPARAM(pTask));
		if (ok == TRUE)
		{
			//DUITRACE(_T("ThreadBase post message ok (%s)"), taskInfo);
			break;
		}

		//DUITRACE(_T("ThreadBase post message fail (%s)"), taskInfo);
		::Sleep(100);
		count -= 1;
	}

	//DUITRACE(_T("ThreadBase post message finish (%s)"), taskInfo);
	return num;
}

void CThreadBase::Suspend()
{
	::SuspendThread(m_HThread);
	m_bIsSuspend = true;
}

void CThreadBase::Resume()
{
	if (m_bIsSuspend == true)
	{
		::ResumeThread(m_HThread);
		m_bIsSuspend = false;
	}
}

void CThreadBase::OnFinishOneTask()
{
}

std::wstring CThreadBase::GetName()
{
	return m_Name;
}

TaskNumber CThreadBase::GenerateTaskNumber()
{
	// �����ȱ��浽�ֲ�������
	TaskNumber num = 0;

	::WaitForSingleObject(m_hMutex, INFINITE);
	m_TopTaskNumber += 1;
	num = m_TopTaskNumber;
	::ReleaseMutex(m_hMutex);

	return num;
}