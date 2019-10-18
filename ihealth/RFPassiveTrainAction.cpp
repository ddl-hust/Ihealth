#include "StdAfx.h"
#include <mmsystem.h>
#include <io.h>
#include "RFPassiveTrainAction.h"
#include "RFMainWindow.h"
#include "RFPassiveTrain.h"
#include "RFPatientsTrainDetails.h"

int FormatTimeValue(std::wstring time)
{
	LPTSTR pstr = NULL;

	int minute = _tcstol(time.c_str(), &pstr, 10);
	int second = _tcstol(pstr + 1, &pstr, 10);

	return minute * 60 * 1000 + second * 1000;
}

// 这是在运行被动运动时的timer，每200ms运行一次。
void OnTimer(HWND hWnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime) {
	RFPassiveTrainAction* action = &RFMainWindow::MainWindow->m_passive_train_action;	
	if (!action) {
		return;
	}

	// 动作开始前播放动作的提示音
	std::wstring mediapath = action->m_curmedia.pathFileName;
	if (action->m_timeplay == 0 && !mediapath.empty() && _waccess(mediapath.c_str(), 0) != -1) {
		sndPlaySound(action->m_curmedia.pathFileName.c_str(), SND_ASYNC);
	}

	action->m_timeplay += 200;
	MEDIA media = action->m_curmedia;
	int timelen = FormatTimeValue(media.train.timelen);
	if (action->m_timeplay < timelen) {
		// 当一个动作没有执行完的时候，需要随时更新动作进行的时间
		RFMainWindow::MainWindow->SetPassiveTrainProgress(action->m_timeplay, timelen, true);
	} else {
		// 动作执行完后，需要将当前训练数据保存到数据库，并开始下一个数据
		if (!RFMainWindow::MainWindow->m_robot.PassiveIsBusy()) {
			action->SaveMovingData();
			if (action->m_medias.size() > 0) {
				if (action->m_orderplay) {
					action->m_curmedia = action->PopOrderMedia();
				} else {
					action->m_curmedia = action->PopRandomMedia();
				}

				action->m_timeplay = 0;
				RFMainWindow::MainWindow->SetPassiveTrainProgress(0, FormatTimeValue(action->m_curmedia.train.timelen), true);
			} else {
				RFMainWindow::MainWindow->SetPassiveTrainProgress(0, FormatTimeValue(action->m_curmedia.train.timelen), false);
				action->StopPlay();
			}
		}
	}

	CLabelUI* pLabel = static_cast<CLabelUI*>(RFMainWindow::MainWindow->m_pm.FindControl(_T("passsive_train_cur_action_name")));
	pLabel->SetText(action->m_curmedia.name.c_str());
}

RFPassiveTrainAction::RFPassiveTrainAction(void) {
	m_orderplay = 0;
	m_timeplay = 0;
	m_isPlaying = false;
	m_currenttimer = 0;

	m_movement_createtime = 0;
}

RFPassiveTrainAction::~RFPassiveTrainAction(void) { }

void RFPassiveTrainAction::StartPlay(std::list<MEDIA>& medias, bool orderplay)
{
	if (medias.size() < 1) {
		return;
	}

	m_medias_old.clear();
	m_medias.clear();

	m_isPlaying = true;
	m_medias_old = medias;
	m_medias = medias;
	m_orderplay = orderplay;
	m_timeplay = 0;
	if (orderplay) {
		m_curmedia = PopOrderMedia();
	} else {
		m_curmedia = PopRandomMedia();
	}
	
	m_currenttimer = ::SetTimer(NULL, RF_TIMER_ID, RF_TIMER_ELAPSE, (TIMERPROC)OnTimer);
}

void RFPassiveTrainAction::StopPlay()
{
	m_timeplay = 0;
	m_isPlaying = false;
	RFMainWindow::MainWindow->m_robot.PassiveStopMove();
	::KillTimer(NULL, m_currenttimer);
	RFMainWindow::MainWindow->SetPassiveTrainProgress(m_timeplay, FormatTimeValue(_T("01:40")), true);
}

void RFPassiveTrainAction::PlayNext(bool orderplay)
{
	if (m_medias.size() < 1) {
		return;
	}

	m_orderplay = orderplay;
	if (m_orderplay) {
		m_curmedia = PopOrderMedia();
	} else {
		m_curmedia = PopRandomMedia();
	}

	m_timeplay = 0;
	RFMainWindow::MainWindow->SetPassiveTrainProgress(0, FormatTimeValue(m_curmedia.train.timelen), true);
}

void RFPassiveTrainAction::PlayPrev(bool orderplay)
{
	if (m_medias.size() < 1) {
		return;
	}

	m_orderplay = orderplay;
	
	MEDIA play = m_curmedia;
	std::list<MEDIA>::iterator begin = m_medias_old.begin();
	for (; begin != m_medias_old.end(); begin++) {
		if (begin->train.id == m_curmedia.train.id) {
			break;
		}

		play = *begin;
	}

	m_curmedia = play;
	m_timeplay = 0;
	RFMainWindow::MainWindow->SetPassiveTrainProgress(0, FormatTimeValue(m_curmedia.train.timelen), true);
}

void RFPassiveTrainAction::SetPlayOrder(bool orderplay)
{
	m_orderplay = orderplay;
}

MEDIA RFPassiveTrainAction::PopRandomMedia()
{
	if (m_medias.size() < 2) {
		return PopOrderMedia();
	}

	int nRandomIndex = 0;
	int num = m_medias.size()-1;
	if (num > 0) {
		srand(timeGetTime());
		nRandomIndex = rand()%num;
	}
	
	MEDIA media;

	int i = 0;
	std::list<MEDIA>::iterator begin = m_medias.begin();
	for (; begin != m_medias.end(); begin++) {
		if (i == nRandomIndex) {
			m_movement_createtime = time(NULL);
			media = *begin;
			m_medias.erase(begin);
			break;
		}

		i++;
	}

	if (RFPassiveTrain::get()->m_robot_indexs.find(media.train.id) != RFPassiveTrain::get()->m_robot_indexs.end()) {
		int index = RFPassiveTrain::get()->m_robot_indexs[media.train.id];
		RFMainWindow::MainWindow->m_robot.PassiveStopMove();
		::Sleep(300U);
		RFMainWindow::MainWindow->m_robot.PassiveStartMove(index);
	} else {
		PopRandomMedia();
	}
	return media;
}

MEDIA RFPassiveTrainAction::PopOrderMedia()
{
	MEDIA media;

	if (m_medias.size() < 1)
	{
		return media;
	}

	std::list<MEDIA>::iterator begin = m_medias.begin();
	m_movement_createtime = time(NULL);
	media = *begin;
	m_medias.erase(begin);

	if (RFPassiveTrain::get()->m_robot_indexs.find(media.train.id) != RFPassiveTrain::get()->m_robot_indexs.end()) {
		int index = RFPassiveTrain::get()->m_robot_indexs[media.train.id];
		RFMainWindow::MainWindow->m_robot.PassiveStopMove();
		::Sleep(300U);
		RFMainWindow::MainWindow->m_robot.PassiveStartMove(index);
	} else {
		PopOrderMedia();
	}

	return media;
}

void RFPassiveTrainAction::SaveMovingData()
{
	time_t finish_time = time(NULL);

	PatientTrainDetails details;// = m_curmedia.train;

	details.patientid = RFMainWindow::MainWindow->m_current_patient.id;
	details.traindate = RFToDateString(m_movement_createtime);
	details.content = RF_TRAINTYPE_STRING_BD;
	details.content += _T("-");
	details.content += m_curmedia.name;
	details.duration = RFToTimeString(m_movement_createtime) + _T("-") + RFToTimeString(finish_time);
	details.traintime = m_curmedia.train.timelen;
	details.createtime = RFToDateTimeString(m_movement_createtime);
	details.traintype = RF_TRAINTYPE_STRING_BD;

	details.lasttreattime = RFToDateString(finish_time);
	details.totaltreattime = details.traintime;
	details.recoverdetail = details.content;

	PassiveData m;
	RFMainWindow::MainWindow->m_robot.PassiveGetCurrentMove(m);

	//details.target_pos[0] = teach.target_positions[0];
	//details.target_pos[1] = teach.target_positions[1];
	details.target_vel[0] = m.target_velocitys[0];
	details.target_vel[1] = m.target_velocitys[1];
	
	RFPatientsTrainDetails::get()->AddPatientTrainDetails(details);
}
