#include "boundarydetection.h"

#include <math.h>
#include <iostream>
#include <process.h>
#include <tchar.h>
#include <windows.h>

#include "robot.h"
#include "data_acquisition.h"

#define BOYDET_TIME 0.1
#define ShoulderTorqueLimit 100.0
#define ElbowTorqueLimit 100.0

#define PullLimit 6.0 /*ʵ�ʵ�ѹ��Ҫ����2*/

double rawTorqueData[5] = {0};
double raw_pull_data[20] = {0};

const char *TCH = "Dev2/ai4:5"; //���زɼ�ͨ��
const char *pull_sensor_channel = "Dev2/ai0:3";

double boundaryDetection::shoulder_torque = 0.0;
double boundaryDetection::elbow_torque = 0.0;


HHOOK hHook;
LRESULT __stdcall CBTHookProc(long nCode, WPARAM wParam, LPARAM lParam)
{
    if (nCode == HCBT_ACTIVATE) {
        SetDlgItemText((HWND)wParam, IDOK, L"ֹͣ");
        UnhookWindowsHookEx(hHook);
    }
    return 0;
}
boundaryDetection::boundaryDetection()
{
    for (int i = 0; i < 4; i++) {
        Pull_Sensor[i] = 0;
        Travel_Switch[i] = 0;
    }
    for (int j = 0; j < 2; j++) {
        angle[j] = 0;
        Torque_Sensor[j] = 0;
        vel[j] = 0;
    }
    for (int k = 0; k < 3; k++) {
        m_Pos_A[k] = 0;
        m_Pos_S[k] = 0;
    }
    m_emergency_stop_status = true;
    vel_i = 0;
    m_stop = false;
    //����һ�������Ļ��������Ϊ���ź�״̬��
    hMutex = CreateMutex(NULL, FALSE, NULL);
    hAngleMutex = CreateMutex(NULL, FALSE, NULL);
    hVelMutex = CreateMutex(NULL, FALSE, NULL);
}
boundaryDetection::~boundaryDetection() { stopBydetect(); }
unsigned int __stdcall BydetectThreadFun(PVOID pParam)
{
    boundaryDetection *Bydetect = (boundaryDetection *)pParam;
    UINT oldTickCount, newTickCount;
    oldTickCount = GetTickCount();
    while (TRUE) {
        if (Bydetect->m_stop) break;
        //��ʱ BOYDET_TIME s
        while (TRUE) {
            newTickCount = GetTickCount();
            if (newTickCount - oldTickCount >= BOYDET_TIME * 1000) {
                oldTickCount = newTickCount;
                break;
            }
            else {
                SwitchToThread();
                ::Sleep(5);
            }
        }
        if (Bydetect->m_stop) break;

        WaitForSingleObject(Bydetect->hMutex, INFINITE);
        Bydetect->getSensorData();
        ReleaseMutex(Bydetect->hMutex);

        WaitForSingleObject(Bydetect->hAngleMutex, INFINITE);
        Bydetect->getEncoderData();
        ReleaseMutex(Bydetect->hAngleMutex);

        WaitForSingleObject(Bydetect->hVelMutex, INFINITE);
        Bydetect->getJointVel();
        ReleaseMutex(Bydetect->hVelMutex);

        Bydetect->getTorqueData();
        Bydetect->GetPullSensorData();

        Bydetect->check();
    }
    return 0;
}
void boundaryDetection::getSensorData()
{
    I32 DI_Group = 0; // If DI channel less than 32
    I32 DI_Data = 0; // Di data
    I32 di_ch[ControlCard::InputChannels];
    I32 returnCode = 0; // Function return code
    returnCode = APS_read_d_input(0, DI_Group, &DI_Data);
    for (int i = 0; i < ControlCard::InputChannels; i++) di_ch[i] = ((DI_Data >> i) & 1);

    Travel_Switch[0] = di_ch[16]; // ORG�ź�-�ⲿ���
    Travel_Switch[1] = di_ch[17]; // MEL�ź�-�ⲿ���

    Travel_Switch[2] = di_ch[18]; // ORG�ź�-�粿���
    Travel_Switch[3] = di_ch[19]; // MEL�ź�-�粿���

    m_emergency_stop_status = di_ch[20];
}
bool *boundaryDetection::GetSwithData()
{
    WaitForSingleObject(hMutex, INFINITE);
    bool *output = Travel_Switch;
    ReleaseMutex(hMutex);

    return output;
}
double *boundaryDetection::getAngle()
{
    double *output = NULL;
    WaitForSingleObject(hAngleMutex, INFINITE);
    output = angle;
    ReleaseMutex(hAngleMutex);
    return output;
}
void boundaryDetection::startBydetect()
{
    HANDLE handle;
    handle = (HANDLE)_beginthreadex(NULL, 0, BydetectThreadFun, this, 0, NULL);
    // WaitForSingleObject(handle, INFINITE);
}
void boundaryDetection::stopBydetect()
{
    m_stop = true;
    //_endthreadex(m_Handle);
}
void boundaryDetection::getTorqueData()
{
    TaskHandle taskHandle = 0;
    int32 read = 0;
    int status = 0;
    status = DAQmxCreateTask("TorqueDataTask", &taskHandle);
    status = DAQmxCreateAIVoltageChan(taskHandle, TCH, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);

    status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);

    status = DAQmxStartTask(taskHandle);
    status = DAQmxReadAnalogF64(taskHandle, 10, 0.2, DAQmx_Val_GroupByScanNumber, rawTorqueData, 20, &read, NULL);
    status = DAQmxStopTask(taskHandle);
    status = DAQmxClearTask(taskHandle);

    for (int j = 0; j < 2; j++) {
        Torque_Sensor[j] = rawTorqueData[j] * 2;
    }
}

void boundaryDetection::GetPullSensorData()
{
    TaskHandle taskHandle = 0;
    int32 read = 0;
    int status = 0;

    status = DAQmxCreateTask("PullDataTask", &taskHandle);
    status = DAQmxCreateAIVoltageChan(taskHandle, pull_sensor_channel, "PullDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
    status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
    status = DAQmxStartTask(taskHandle);
    status = DAQmxReadAnalogF64(taskHandle, 5, 0.2, DAQmx_Val_GroupByScanNumber, raw_pull_data, 20, &read, NULL);
    status = DAQmxStopTask(taskHandle);
    status = DAQmxClearTask(taskHandle);
    for (int i = 0; i < 4; i++) {
        Pull_Sensor[i] = raw_pull_data[i] * 2;
    }
}

void boundaryDetection::getTorqueData(double data[2])
{
    TaskHandle taskHandle = 0;
    int32 read = 0;
    int status = 0;
    status = DAQmxCreateTask("TorqueDataTask", &taskHandle);
    status = DAQmxCreateAIVoltageChan(taskHandle, TCH, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);

    status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);

    status = DAQmxStartTask(taskHandle);
    status = DAQmxReadAnalogF64(taskHandle, 10, 0.2, DAQmx_Val_GroupByScanNumber, rawTorqueData, 20, &read, NULL);
    status = DAQmxStopTask(taskHandle);
    status = DAQmxClearTask(taskHandle);

    for (int j = 0; j < 2; j++) {
        data[j] = rawTorqueData[j] * 2;
    }
}
void boundaryDetection::getEncoderData()
{
    int ret = 0;
    double raw_arm = 0;
    double raw_shoulder = 0;
    ret = APS_get_position_f(ControlCard::ElbowAxisId, &raw_arm);
    ret = APS_get_position_f(ControlCard::ShoulderAxisId, &raw_shoulder);
    angle[0] = raw_shoulder * ControlCard::Unit_Convert;
    angle[1] = raw_arm * ControlCard::Unit_Convert;
}

void boundaryDetection::getJointVel()
{
    if (vel_i >= 3) {
        vel_i = 0;
    }
    WaitForSingleObject(hAngleMutex, INFINITE);
    m_Pos_S[vel_i] = angle[0]; //�粿�Ƕ�ֵ
    m_Pos_A[vel_i] = angle[1]; //�ⲿ�Ƕ�ֵ
    ReleaseMutex(hAngleMutex);
    //��λ�����ٶȵ��㷨

    //������õ��ٶȣ�������BOYDET_TIME���ڵ�λ�Ƴ���ʱ��
    switch (vel_i) {
    case 0:
        (m_Pos_A[1] == 0) ? vel[1] = 0 : vel[1] = (m_Pos_A[0] - m_Pos_A[1]) / (BOYDET_TIME * 2);
        (m_Pos_S[1] == 0) ? vel[0] = 0 : vel[0] = (m_Pos_S[0] - m_Pos_S[1]) / (BOYDET_TIME * 2);
        break;
    case 1:
        (m_Pos_A[2] == 0) ? vel[1] = 0 : vel[1] = (m_Pos_A[1] - m_Pos_A[2]) / (BOYDET_TIME * 2);
        (m_Pos_S[2] == 0) ? vel[0] = 0 : vel[0] = (m_Pos_S[1] - m_Pos_S[2]) / (BOYDET_TIME * 2);
        break;
    case 2:
        vel[1] = (m_Pos_A[2] - m_Pos_A[0]) / (BOYDET_TIME * 2);
        vel[0] = (m_Pos_S[2] - m_Pos_S[0]) / (BOYDET_TIME * 2);
        break;
    }
    vel_i++;
}
double *boundaryDetection::getVel()
{
    WaitForSingleObject(hVelMutex, INFINITE);
    double *output = vel;
    ReleaseMutex(hVelMutex);
    return output;
}

void boundaryDetection::check()
{
    //��ͣ������ʾ
    // if (!m_emergency_stop_status) {
    // 	int ret = ::MessageBox(m_hWnd, _T("��⵽��ͣ���ر����£���������Ƿ���������, ���ȷ���ر������"), _T("Ӳ����ͣ"), MB_OK | MB_ICONEXCLAMATION);
    // 	if (ret == IDOK) {
    // 		ControlCard::GetInstance().Close();
    // 		::PostMessage(m_hWnd, WM_QUIT, NULL, NULL);
    // 	}
    // }

    //// ���ر���
    // DataAcquisition::GetInstance().AcquisiteTorqueData();
    // shoulder_torque = DataAcquisition::GetInstance().ShoulderTorque();
    // elbow_torque = DataAcquisition::GetInstance().ElbowTorque();
    // double abs_shoulder_torque = fabs(shoulder_torque);
    // double abs_elbow_torque = fabs(elbow_torque);
    ////AllocConsole();
    ////freopen("CONOUT$", "w", stdout);
    ////printf("%lf    %lf    \n", abs_shoulder_torque, abs_elbow_torque);
    // if (abs_shoulder_torque > ShoulderTorqueLimit || abs_elbow_torque > ElbowTorqueLimit) {
    //	//������Ҫ�ȰѶ�����ͣ���������Ǿ���post message�ķ�ʽȥ��ͣ��ֱ�ӵ�����ͣ�Ľӿڡ�
    //	::PostMessage(m_hWnd, TorqueError, NULL, NULL);
    //	// Ȼ����ʾһ��MessageBoxȥ��ʾ��λ
    //	hHook = SetWindowsHookEx(WH_CBT, (HOOKPROC)CBTHookProc, NULL, GetCurrentThreadId());
    //	int ret = ::MessageBox(m_hWnd, _T("�ؽ����س�����ɷ�Χ����ҽ����黼���Ƿ������Ρ�"), _T("���ر���"), MB_OK | MB_ICONEXCLAMATION);
    //	if (ret == IDOK) {
    //		ControlCard::GetInstance().ResetPosition();
    //	}
    //}

    // ��������
    DataAcquisition::GetInstance().AcquisitePullSensorData();
    double abs_shoulder_forward_pull = fabs(DataAcquisition::GetInstance().ShoulderForwardPull());
    double abs_shoulder_backward_pull = fabs(DataAcquisition::GetInstance().ShoulderBackwardPull());
    double abs_elbow_forward_pull = fabs(DataAcquisition::GetInstance().ElbowForwardPull());
    double abs_elbow_backward_pull = fabs(DataAcquisition::GetInstance().ElbowBackwardPull());
    if (abs_shoulder_forward_pull > PullLimit || abs_shoulder_backward_pull > PullLimit || abs_elbow_forward_pull > PullLimit
        || abs_elbow_backward_pull > PullLimit) {
        // ͬ����Ҫ�ȰѶ�����ͣ����
        ::PostMessage(m_hWnd, PullForceError, NULL, NULL);
        wstring msg(_T("��˿������������ɷ�Χ�������쳣,F1="));
        msg += to_wstring(abs_shoulder_forward_pull);
        msg += (_T(", F2="));
        msg += to_wstring(abs_shoulder_backward_pull);
        msg += (_T(", F3="));
        msg += to_wstring(abs_elbow_forward_pull);
        msg += (_T(", F4="));
        msg += to_wstring(abs_elbow_backward_pull);
        msg += (_T("��"));
        // Ȼ����ʾһ��MessageBoxȥ��ʾ��λ
        hHook = SetWindowsHookEx(WH_CBT, (HOOKPROC)CBTHookProc, NULL, GetCurrentThreadId());
        int ret = ::MessageBox(m_hWnd, msg.c_str(), _T("��������"), MB_OK | MB_ICONEXCLAMATION);
        if (ret == IDOK) {
            // ControlCard::GetInstance().ResetPosition();
            m_pRobot->ActiveStopMove();
        }
    }
    // AllocConsole();
    // freopen("CONOUT$", "w", stdout);
    // printf("shoulder:%lf    %lf \n elbow:%lf     %lf\n", abs_shoulder_forward_pull,
    // abs_shoulder_backward_pull,abs_elbow_forward_pull,abs_elbow_backward_pull);
}


void boundaryDetection::Set_hWnd(HWND hWnd) { m_hWnd = hWnd; }

void boundaryDetection::SetRobot(Robot *pRobot) { m_pRobot = pRobot; }
