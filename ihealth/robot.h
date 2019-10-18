#pragma once
#include "passive_control.h"
#include "boundarydetection.h"
#include "control_card.h"
#include "active_control.h"
#include "emgcontrl.h"
#include "EyeMode.h"
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>


class Robot
{
public:
    Robot();
    ~Robot();
    /************************************************************************/
    /*                         passive mode                               */
    /************************************************************************/
    /**
     * brief begin passive motion
     * parm @ index :the index of motion
     */
    void PassiveStartMove(int index);
    void PassiveStopMove();
    void PassiveBeginRecord();
    void PassiveStopRecord();
    /**
     * brief return the latest passive motion
     * parm @ move :store the latest motion
     */
    void PassiveGetCurrentMove(PassiveData &move);
    /**
     * brief return the latest passive motion record(teach motion)
     * parm @ teach :store the latest motion record
     */
    void PassiveGetCurrentRecord(PassiveData &teach);
    void PassiveClearMovementSet(); // clear all passive motion set
    void PassiveStoreCurrentRecord(); // store current teach motion to motion set
    /**
     * brief store the specfic motion to motion set
     * parm @ move :the motion you want to store 
     */
    void PassiveStoreMovement(const PassiveData &move);
    bool PassiveIsBusy(); // check is passive motion is running
    bool IsPassiveRecording(); //check is passive record is running

    /************************************************************************/
    /*                           active mode                                */
    /************************************************************************/
    void ActiveStartMove();
    void ActiveStartMove(int id); //refactor for temp paitent data export solution
    void ActiveStopMove();
    double GetGripStrength();
    void GetPlanePos(short w, short h, double XY[2]);
    void CalculateRagPos(double XY[2]);
    void SetDamping(float FC = 0.1);
    // void ExportJointData(int id);
    void SetPressureSensorOn(); //主动模式纯六维力-压力+六维力切换相�?
	void SetPressureSensorOff();

    /************************************************************************/
    /*                           emg mode                                 */
    /************************************************************************/
    bool EMGIsMove();
    void EMGStartMove();
    void EMGStopMove();
    double EMGGetSignal(int index = 0);

    /************************************************************************/
    /*                           eye mode                                */
    /************************************************************************/
    void enterEyeMode(); // call it while enter eye mode.
    void exitEyeMode(); // call it while enter eye mode.
    void getLeftRGB24(unsigned char *data, int _width, int _height); // get image data of left eye
    void getRightRGB24(unsigned char *data, int _width, int _height); // get image data of right eye
    void startEyeMove(); // call it while clicking the start
    void stopEyeMove(); // call it while clicking the stop
    void setEyeVel(double factor); // set velocity
    void eyeCalibrate(); // call it before startEyeControl.
    void resetPos();
    void stopResetPos();
    void setWindow(HWND hWnd);

public:
    PassiveControl *pasvMode; 
    boundaryDetection *bDetect;
    ActiveControl *activeCtrl;
    emgcontrl *EMGContrl;
    EyeMode *eyeModeCtl;

    HWND m_hWnd = NULL;
    bool m_isPasvModeStart;
    bool m_isActiveModeStart;
    bool m_isEmgModeStart;
};

void getSensorData(bool Travel_Switch[4]);
