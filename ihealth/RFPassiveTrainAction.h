#pragma once
#include "MusicPlayer.h"

#define RF_TIMER_ELAPSE 200U	// 10HZ
#define RF_TIMER_ID	888



class RFPassiveTrainAction
{
public:
	RFPassiveTrainAction(void);
	~RFPassiveTrainAction(void);

	//void OnTimer(HWND hWnd, UINT uMsg, UINT_PTR idEvent, DWORD dwTime);
	void StartPlay(std::list<MEDIA>& medias, bool orderplay);
	void StopPlay();

	void PlayNext(bool orderplay);
	void PlayPrev(bool orderplay);

	void SetPlayOrder(bool orderplay);

	// 随机运行被动动作
	MEDIA PopRandomMedia();
	// 按照列表顺序依次运行被动动作
	MEDIA PopOrderMedia();

	void SaveMovingData();
	time_t			m_movement_createtime;

	std::list<MEDIA> m_medias_old;

	std::list<MEDIA>	m_medias;
	bool				m_orderplay;
	MEDIA				m_curmedia;
	int					m_timeplay;

	bool				m_isPlaying;

	UINT_PTR	m_currenttimer;
};
