#include "robot.h"

#include <windows.h>
#include <mmsystem.h>

#include "data_acquisition.h"
#include "Log.h"

#pragma comment(lib, "winmm.lib")
#define RESET_TIMER 100
MMRESULT Mtimer_ID = 0;
UINT wAccuracy = 0;

Robot::Robot()
{
    bDetect = nullptr;
    bDetect = new boundaryDetection();
    bDetect->SetRobot(this);

    activeCtrl = nullptr;
    activeCtrl = new ActiveControl();

    pasvMode = nullptr;
    pasvMode = new PassiveControl();
    pasvMode->SetActiveControl(activeCtrl);

    ControlCard::GetInstance().Initial();

    EMGContrl = nullptr;
    EMGContrl = new emgcontrl();

    eyeModeCtl = nullptr;
    eyeModeCtl = new EyeMode(bDetect);

    bDetect->startBydetect();
    m_isActiveModeStart = false;
    m_isEmgModeStart = false;
    m_isPasvModeStart = false;
}

Robot::~Robot()
{
    if (pasvMode != NULL) delete pasvMode;
    if (bDetect != NULL) {
        delete bDetect;
    }
    if (NULL != activeCtrl) delete activeCtrl;
    if (NULL != EMGContrl) delete EMGContrl;
    if (NULL != eyeModeCtl) delete eyeModeCtl;
}

unsigned __stdcall PositionResetThread(void *)
{
    ControlCard::GetInstance().ResetPosition();
    return 0;
}

void Robot::PassiveClearMovementSet() { pasvMode->ClearMovementSet(); }

void Robot::PassiveStoreMovement(const PassiveData &move) { pasvMode->StoreMovement(move); }

bool Robot::PassiveIsBusy() { return pasvMode->IsBusy(); }

void Robot::PassiveStartMove(int index)
{
    if (m_isPasvModeStart == false) {
        m_isPasvModeStart = true;
        pasvMode->BeginMove(index);
    }
}

void Robot::PassiveStopMove()
{
    if (m_isPasvModeStart == true) {
        m_isPasvModeStart = false;
        pasvMode->StopMove();
    }
}
void Robot::PassiveGetCurrentMove(PassiveData &teach) { pasvMode->GetCurrentMove(teach); }

void Robot::PassiveBeginRecord() { pasvMode->BeginRecord(); }

void Robot::PassiveStopRecord() { pasvMode->StopRecord(); }

void Robot::PassiveGetCurrentRecord(PassiveData &teach) { pasvMode->GetCurrentRecord(teach); }

void Robot::PassiveStoreCurrentRecord() { pasvMode->StoreCurrentRecord(); }

void Robot::ActiveStartMove()
{
    if (!m_isActiveModeStart) {
        activeCtrl->StartMove();
        m_isActiveModeStart = true;
    }
}

void Robot::ActiveStopMove()
{
    if (m_isActiveModeStart) {
        activeCtrl->StopMove();
        m_isActiveModeStart = false;
    }
}

double Robot::GetGripStrength()
{
    double output;
    DataAcquisition::GetInstance().AcquisiteGripData(&output);
    return output;
}

bool Robot::IsFire() { return activeCtrl->IsFire(); }

void Robot::GetPlanePos(short w, short h, double XY[2]) { activeCtrl->CalculatePlaneXY(w, h, XY); }

void Robot::CalculateRagPos(double XY[2]) { activeCtrl->CalculateRagXY(XY); }

void Robot::SetDamping(float FC /* =0.1 */) { activeCtrl->SetDamping(FC); }


void Robot::setEyeVel(double factor) { eyeModeCtl->setVel(factor); }

void Robot::eyeCalibrate() { eyeModeCtl->calibrate(); }

void Robot::startEyeMove() { eyeModeCtl->start(); }

void Robot::stopEyeMove() { eyeModeCtl->stop(); }

void Robot::enterEyeMode() { eyeModeCtl->enter(); }

void Robot::exitEyeMode() { eyeModeCtl->exit(); }

void Robot::getLeftRGB24(unsigned char *data, int _width, int _height) { eyeModeCtl->getRGB24(data, _width, _height, EyeMode::LEFT); }

void Robot::getRightRGB24(unsigned char *data, int _width, int _height) { eyeModeCtl->getRGB24(data, _width, _height, EyeMode::RIGHT); }

void Robot::resetPos() { _beginthreadex(NULL, 0, PositionResetThread, NULL, 0, NULL); }

void Robot::setWindow(HWND hWnd)
{
    m_hWnd = hWnd;
    bDetect->Set_hWnd(hWnd);
    pasvMode->SetHWND(hWnd);
    ControlCard::GetInstance().Set_hWnd(hWnd);
    EMGContrl->m_hWnd = hWnd;
}

bool Robot::EMGIsMove() { return EMGContrl->isBeginMove; }

void Robot::EMGStartMove()
{
    // if (ctrlCard->IsCardInitial()) {
    if (m_isEmgModeStart == false) {
        m_isEmgModeStart = true;
        EMGContrl->start(bDetect);
    }
    //}
}
void Robot::EMGStopMove()
{
    // if (ctrlCard->IsCardInitial()) {
    if (m_isEmgModeStart == true) {
        m_isEmgModeStart = false;
        EMGContrl->stop();
    }
    //}
}

double Robot::EMGGetSignal(int index /* = 0 */) { return EMGContrl->getRawData(index); }

void Robot::stopResetPos()
{
    if (Mtimer_ID != 0) timeKillEvent(Mtimer_ID);
    if (wAccuracy != 0) timeEndPeriod(wAccuracy);
}


void getSensorData(bool Travel_Switch[4])
{
    I32 DI_Group = 0; // If DI channel less than 32
    I32 DI_Data = 0; // Di data
    I32 di_ch[ControlCard::InputChannels];
    I32 returnCode = 0; // Function return code
    returnCode = APS_read_d_input(0, DI_Group, &DI_Data);
    for (int i = 0; i < ControlCard::InputChannels; i++) di_ch[i] = ((DI_Data >> i) & 1);

    Travel_Switch[0] = di_ch[16]; // 0�ŵ��ORG�ź�-�ⲿ���
    Travel_Switch[1] = di_ch[17]; // 0�ŵ��MEL�ź�-�ⲿ���

    Travel_Switch[2] = di_ch[18]; // 1�ŵ��ORG�ź�-�粿���
    Travel_Switch[3] = di_ch[19]; // 1�ŵ��MEL�ź�-�粿���
}

bool Robot::IsPassiveRecording() { return pasvMode->in_record_status_; }
