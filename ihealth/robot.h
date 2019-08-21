#pragma once
#include "passive_control.h"
#include "boundarydetection.h"
#include "control_card.h"
#include "active_control.h"
#include "emgcontrl.h"
#include "EyeMode.h"


class Robot
{
public:
    Robot();
    ~Robot();
    /************************************************************************/
    /*                           ����ģʽ�ӿ�                                */
    /************************************************************************/
    //��ʼ�����˶���index-��ʾ����������
    void PassiveStartMove(int index);
    //ֹͣ�����˶�
    void PassiveStopMove();
    //��ʼ¼�ƶ���
    void PassiveBeginRecord();
    //����ʾ��
    void PassiveStopRecord();
    // ���������һ�α����˶�
    void PassiveGetCurrentMove(PassiveData &move);
    // ���������һ��¼��
    void PassiveGetCurrentRecord(PassiveData &teach);
    // ��������˶���������
    void PassiveClearMovementSet();
    // �������¼�����ݱ����ڱ����˶�����������
    void PassiveStoreCurrentRecord();
    // ����ָ���ı�������
    void PassiveStoreMovement(const PassiveData &move);
    // ���ر����˶��Ƿ������˶���¼��
    bool PassiveIsBusy();
    // ���ر����˶��Ƿ�����¼��
    bool IsPassiveRecording();

    /************************************************************************/
    /*                           ����ģʽ�ӿ�                                */
    /************************************************************************/
    //��ʼ�����˶�
    void ActiveStartMove();
    //���������˶�
    void ActiveStopMove();
    //��������-���ݽӿ�
    double GetGripStrength();
    bool IsFire();
    void GetPlanePos(short w, short h, double XY[2]);
    // ��������Ϸ���棬��ȡ������XY
    void CalculateRagPos(double XY[2]);
    void SetDamping(float FC = 0.1);

    /************************************************************************/
    /*                           sEMGģʽ�ӿ�                                */
    /************************************************************************/
    bool EMGIsMove();
    //��ʼEMG�˶�
    void EMGStartMove();
    //ֹͣEMG�˶�
    void EMGStopMove();
    //��ȡEMG�ź�-���ݽӿڣ�index-�źű�ţ��ֱ�Ϊ0��1��2��3
    double EMGGetSignal(int index = 0);

    /************************************************************************/
    /*                           �۶�ģʽ�ӿ�                                */
    /************************************************************************/
    //���عؽڽǶ�-���ݽӿ�,0-�粿�ؽڽǶȣ�1-�ⲿ�ؽڽǶ�(ͬ��������ģʽ�ӿ�)
    void enterEyeMode(); // call it while enter eye mode.
    void exitEyeMode(); // call it while enter eye mode.
    void getLeftRGB24(unsigned char *data, int _width, int _height); // get image data of left eye
    void getRightRGB24(unsigned char *data, int _width, int _height); // get image data of right eye
    void startEyeMove(); // call it while clicking the start
    void stopEyeMove(); // call it while clicking the stop
    void setEyeVel(double factor); // set velocity
    void eyeCalibrate(); // call it before startEyeControl.

    //��λ
    void resetPos();
    void stopResetPos();

    void setWindow(HWND hWnd);

public:
    PassiveControl *pasvMode; //��������ģʽ
    boundaryDetection *bDetect; //�߽���
    ActiveControl *activeCtrl;
    emgcontrl *EMGContrl;
    EyeMode *eyeModeCtl;

    HWND m_hWnd = NULL;
    bool m_isPasvModeStart;
    bool m_isActiveModeStart;
    bool m_isEmgModeStart;
};

void getSensorData(bool Travel_Switch[4]);