#pragma once
#include"passive_control.h"
#include "boundarydetection.h"
#include "control_card.h"
#include"active_control.h"
#include"emgcontrl.h"
#include "EyeMode.h"


class Robot {
public:
	Robot();
	~Robot();
	/************************************************************************/
	/*                           被动模式接口                                */
	/************************************************************************/
	//开始被动运动，index-表示动作的索引
	void PassiveStartMove(int index);
	//停止被动运动
	void PassiveStopMove();
	//开始录制动作
	void PassiveBeginRecord();
	//结束示教
	void PassiveStopRecord();
	// 返回最近的一次被动运动
	void PassiveGetCurrentMove(PassiveData& move);
	// 返回最近的一次录制
	void PassiveGetCurrentRecord(PassiveData& teach);
	// 清除被动运动动作序列
	void PassiveClearMovementSet();
	// 将最近的录制数据保存在被动运动动作序列中
	void PassiveStoreCurrentRecord();
	// 保存指定的被动动作
	void PassiveStoreMovement(const PassiveData& move);
	// 返回被动运动是否正在运动或录制
	bool PassiveIsBusy();
	// 返回被动运动是否正在录制
	bool IsPassiveRecording();

	/************************************************************************/
	/*                           主动模式接口                                */
	/************************************************************************/
	//开始主动运动
	void ActiveStartMove(int id);
	//结束主动运动
	void ActiveStopMove();
	//返回握力-数据接口
	double GetGripStrength();
	bool	IsFire();
	void	GetPlanePos(short w, short h, double XY[2]);
	// 擦窗户游戏里面，获取窗户的XY
	void	CalculateRagPos(double XY[2]);
	void	SetDamping(float FC=0.1);
	void    SetPressureSensorOn();
	void    SetPressureSensorOff();

	/************************************************************************/
	/*                           sEMG模式接口                                */
	/************************************************************************/
	bool EMGIsMove();
	//开始EMG运动
	void EMGStartMove();
	//停止EMG运动
	void EMGStopMove();
	//获取EMG信号-数据接口，index-信号编号，分别为0，1，2，3
	double EMGGetSignal(int index = 0);

	/************************************************************************/
	/*                           眼动模式接口                                */
	/************************************************************************/
	//返回关节角度-数据接口,0-肩部关节角度，1-肘部关节角度(同上面主动模式接口) 
	void enterEyeMode(); // call it while enter eye mode.
	void exitEyeMode();  // call it while enter eye mode.
	void getLeftRGB24(unsigned char* data, int _width, int _height);  // get image data of left eye
	void getRightRGB24(unsigned char* data, int _width, int _height); // get image data of right eye
	void startEyeMove(); // call it while clicking the start
	void stopEyeMove();  // call it while clicking the stop
	void setEyeVel(double factor); // set velocity
	void eyeCalibrate(); // call it before startEyeControl.

	//复位
	void resetPos();
	void stopResetPos();

	void setWindow(HWND hWnd);
public:
	PassiveControl *pasvMode;//被动控制模式
	boundaryDetection *bDetect;//边界检测
	ActiveControl *activeCtrl;
	emgcontrl *EMGContrl;
	EyeMode *eyeModeCtl;
	
	HWND m_hWnd = NULL;
	bool m_isPasvModeStart;
	bool m_isActiveModeStart;
	bool m_isEmgModeStart;
};

void getSensorData(bool Travel_Switch[4]);