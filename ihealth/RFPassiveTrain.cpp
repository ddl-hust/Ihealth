#include "StdAfx.h"
#include "RFPassiveTrain.h"
#include "RFMainWindow.h"

RFPassiveTrain* RFPassiveTrain::m_sigleton = NULL;

RFPassiveTrain* RFPassiveTrain::get()
{
	if (m_sigleton == NULL) {
		m_sigleton = new RFPassiveTrain();
	}

	return m_sigleton;
}

void RFPassiveTrain::release()
{
	if (m_sigleton) {
		delete m_sigleton;
		m_sigleton = NULL;
	}
}

RFPassiveTrain::RFPassiveTrain(void)
{
}

RFPassiveTrain::~RFPassiveTrain(void)
{
}

int RFPassiveTrain::LoadPassiveTrainInfo()
{

	CTask::Assign(CTask::NotWait, Panic(), NULL, EventHandle(&RFMySQLThread::LoadPassiveTrainInfo), RFMainWindow::UIThread, RFMainWindow::DBThread);
	return 1;
}

void RFPassiveTrain::AddPassiveTrainInfo(PassiveTrainInfo train)
{
	PassiveTrainInfo* pParam = new PassiveTrainInfo;
	*pParam = train;
	CTask::Assign(CTask::NotWait, Panic(), pParam, EventHandle(&RFMySQLThread::AddPassiveTrainInfo), RFMainWindow::UIThread, RFMainWindow::DBThread);
}

int RFPassiveTrain::OnAddPassiveTrainInfoOK(EventArg* pArg) 
{
	CTask *pTask = pArg->GetAttach<CTask*>();
	PassiveTrainInfo *pParam = pTask->GetContext<PassiveTrainInfo*>();
	if (!pParam) {
		return 1;
	}

	RFPassiveTrain::get()->m_passivetraininfos.push_back(*pParam);

	// ˢ��robot�еĶ����б��������µĶ���
	RFMainWindow::MainWindow->m_robot.PassiveClearMovementSet();
	int index = 0;
	std::list<PassiveTrainInfo>::iterator begin = RFPassiveTrain::get()->m_passivetraininfos.begin();
	for (; begin != RFPassiveTrain::get()->m_passivetraininfos.end(); begin++) {
		PassiveData item;

		item.target_positions[0] = begin->target_pos[0];
		item.target_positions[1] = begin->target_pos[1];
		item.target_velocitys[0] = begin->target_vel[0];
		item.target_velocitys[1] = begin->target_vel[1];

		RFPassiveTrain::get()->m_robot_indexs[begin->id] = index;
		RFMainWindow::MainWindow->m_robot.PassiveStoreMovement(item);

		index++;
	}

	RFMainWindow::MainWindow->ShowPassiveTrainPage();

	delete pParam;
	pParam = NULL;
}

int RFPassiveTrain::OnDeletePassiveTrainInfoOK(EventArg *pArg) {
	CTask *pTask = pArg->GetAttach<CTask *>();
	PassiveTrainInfo *pParam = pTask->GetContext<PassiveTrainInfo*>();
	if (!pParam) {
		return 1;
	}
	
	std::list<PassiveTrainInfo>::iterator begin = RFPassiveTrain::get()->m_passivetraininfos.begin();
	for (auto iter = begin; iter != RFPassiveTrain::get()->m_passivetraininfos.end(); ) {
		if (iter->name == pParam->name) {
			iter = RFPassiveTrain::get()->m_passivetraininfos.erase(iter);
		} else {
			++iter;
		}
	} 
	RFMainWindow::MainWindow->m_robot.PassiveClearMovementSet();

	RFMainWindow::MainWindow->ShowPassiveTrainPage();

	delete pParam;
	pParam = NULL;
}

int RFPassiveTrain::OnLoadPassiveTrainInfoOK(EventArg* pArg)
{
	RFMySQLThread *pCurrentThread = pArg->GetSender<RFMySQLThread*>();
	CTask *pTask = pArg->GetAttach<CTask*>();
	LoadPassiveTrainInfoResult *pParam = pTask->GetContext<LoadPassiveTrainInfoResult*>();
	if (!pParam) {
		return 1;
	}

	RFPassiveTrain::get()->m_passivetraininfos = pParam->passivetraininfos;

	// ˢ��robot�еĶ����б�
	RFMainWindow::MainWindow->m_robot.PassiveClearMovementSet();
	int index = 0;
	std::list<PassiveTrainInfo>::iterator begin = pParam->passivetraininfos.begin();
	for (; begin != pParam->passivetraininfos.end(); begin++) {
		PassiveData teach;

		teach.target_positions[0] = begin->target_pos[0];
		teach.target_positions[1] = begin->target_pos[1];
		teach.target_velocitys[0] = begin->target_vel[0];
		teach.target_velocitys[1] = begin->target_vel[1];
	
		RFPassiveTrain::get()->m_robot_indexs[begin->id] = index;
		RFMainWindow::MainWindow->m_robot.PassiveStoreMovement(teach);

		index++;
	}
	delete pParam;
	return 1;
}