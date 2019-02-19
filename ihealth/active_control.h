#ifndef ACTIVECONTROL_H
#define ACTIVECONTROL_H
#include "control_card.h"

#include "matrix.h"

class ActiveControl {
public:
    ActiveControl();
    ~ActiveControl();
	void LoadParamFromFile();
    void StartMove();
    void StopMove();
	// 采集一次六维力的数据，计算出电机速度，然后指示电机以这个速度运动.这是一轮循环
	void Step();
	bool IsFire();
	// 获取机器人末端位置
	void CalculatePlaneXY(short Axis_X, short Axis_Y, double XY[2]);
	// 擦窗户游戏中获取抹布位置
	void CalculateRagXY(double XY[2]);
	void SetDamping(float FC=0.1);
	// 设置关节运动范围
	void SetSAAMax(double saa);
	void SetSFEMax(double sfe);

private:
	void MoveInNewThread();
	void ExitMoveThread();
	void ActMove();
	//将原始值进行坐标变换
	void Raw2Trans(double RAWData[6], double DistData[6]);
	//将转换后的值进行滤波-二阶巴特沃斯低通滤波器
	void Trans2Filter(double TransData[6], double FiltedData[6]);
	void FiltedVolt2Vel(double FiltedData[6]);

public:
	bool is_exit_thread_;
	bool is_moving_;
	double six_dimension_offset_[6];
	double cycle_time_in_second_;

private:
	HANDLE move_thread_handle_;
	Matrix3d rotate_matrix_;
	//手柄坐标系下手柄坐标系原点到六维力坐标系原点的向量
	Vector3d force_position_;

	double shoulder_angle_max_;
	double elbow_angle_max_;

};

#endif // ACTIVECONTROL_H
