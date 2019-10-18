#pragma once
#include "RFMySQLThread.h"

class RFPassiveTrain
{
public:
	static RFPassiveTrain* get();
	static void release();
	static RFPassiveTrain* m_sigleton;

public:
	RFPassiveTrain(void);
	~RFPassiveTrain(void);

	// 从数据库中加载被动运动的动作数据
	int LoadPassiveTrainInfo();
	// 将被动运动中录制的数据添加到数据库中
	void AddPassiveTrainInfo(PassiveTrainInfo train);
	
	std::map<std::wstring, int> m_robot_indexs;
	std::list<PassiveTrainInfo> m_passivetraininfos;

	static int OnLoadPassiveTrainInfoOK(EventArg* pArg);
	// 把被动录制的数据添加到数据库中之后，需要将当前动作集刷新
	// 并跳转到被动运动主页面
	static int OnAddPassiveTrainInfoOK(EventArg* pArg);
	static int OnDeletePassiveTrainInfoOK(EventArg *pArg);
};
