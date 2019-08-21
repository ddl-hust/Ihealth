#pragma once
#include "RFMySQLThread.h"

class RFPassiveTrain
{
public:
    static RFPassiveTrain *get();
    static void release();
    static RFPassiveTrain *m_sigleton;

public:
    RFPassiveTrain(void);
    ~RFPassiveTrain(void);

    // �����ݿ��м��ر����˶��Ķ�������
    int LoadPassiveTrainInfo();
    // �������˶���¼�Ƶ��������ӵ����ݿ���
    void AddPassiveTrainInfo(PassiveTrainInfo train);

    std::map<std::wstring, int> m_robot_indexs;
    std::list<PassiveTrainInfo> m_passivetraininfos;

    static int OnLoadPassiveTrainInfoOK(EventArg *pArg);
    // �ѱ���¼�Ƶ��������ӵ����ݿ���֮����Ҫ����ǰ������ˢ��
    // ����ת�������˶���ҳ��
    static int OnAddPassiveTrainInfoOK(EventArg *pArg);
    static int OnDeletePassiveTrainInfoOK(EventArg *pArg);
};
