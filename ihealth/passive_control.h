#ifndef PASVCONTRL_H
#define PASVCONTRL_H
#include"control_card.h"
#include"boundarydetection.h"
#include"active_control.h"
#include <vector>
#include <queue>
#include <string>
#include <vector>

// 这个结构体用于保存被动运动数据或者被动录制数据
struct PassiveData {
	std::vector<double> target_positions[2];//存储位置的数组
	std::vector<double> target_velocitys[2];//存储速度的数组
	std::vector<double>Interpolation_Data[2];//存储插值后的位置数组
	double round_time;//运动时间（单位为s）
};


// 控制被动运动的类，控制被动运动的录制，以及根据录制好的被动
// 运动进行运动
class PassiveControl {
public:
    PassiveControl();
    ~PassiveControl();

	// 进行被动运动，index表示被动运动的索引
	void BeginMove(int index);
	// 进行一步被动运动，一个完整的被动运动分为多个steps
	// 每延时一段时间就运行一个step
	void MoveStep();
	// 被动运动停止
	void StopMove();
	//录制运动信息
	void SampleStep();
	void GetCurrentMove(PassiveData& move);

	// 开始录制动作
	void BeginRecord();
	// 进行一步录制动作，一个完整的录制动作分为多个steps
	// 每隔一段时间就运行一个step
	void RecordStep();
	// 录制停止
	void StopRecord();
	// 获取最近录制的运动数据，record是输出参数
	void GetCurrentRecord(PassiveData& record);

	// 加入新的示教数据到运动序列中（运动序列包含多条示教数据，示教数据包含角度和速度的数组）
	void StoreMovement(const PassiveData& movement);
	// 添加新动作到运动序列中
	void StoreCurrentRecord();
	// 清除被动运动序列的数据
	void ClearMovementSet();

	//在示教后将曲线置换成正弦曲线
	void CruveSmoothing();
	//获取数组内的最大值
	double GetMaxData(std::vector<double> &data);
	//画sin曲线
	void DrawSincruve();
	//控制是否运行替换sin曲线函数
	bool is_teach;

	//输出txt文件
	void InterpolationTraceExport();//插值后的轨迹
	void PracticalTraceExport();//实际的轨迹
	void TeachPosData();//录制的位置

	// 正在进行录制或者被动运动
	bool IsBusy();

	void SetHWND(HWND hWnd);
	void SetActiveControl(ActiveControl *p);

private:
	// hermite插值算法-返回值为t时间点下电机的位置
	double PHermite(double foretime[2],	//已知时间点
					double forepos[2],	//已知位置点
					double forevel[2],	//已知速度点
					double t);			//所求时间点


public:
	bool in_record_status_;
	bool in_move_status_;
	bool is_busy_;

	std::vector<PassiveData> movement_set_; // 存储所有被动运动数据
private:
	PassiveData record_data_;
	PassiveData move_data_;
	PassiveData sample_data_;

	ActiveControl *active_control_ = nullptr;

	HWND hWnd_ = NULL;
    int hermite_time_counter_ = 0;
	int hermite_target_counter_ = 0;
	double hermite_time_ = 0.0;
	double hermite_time_interval_[2][2];// 插值过程中的时间区间
	double  hermite_vel_interval_[2][2];// 插值过程中的速度区间
	double  hermite_pos_interval_[2][2];// 插值过程中的位置区间

	double max_pos[2];//储存位置的最大值
	int array_size;//数组的个数，需要为奇数
	int curve_x;//用于求sin曲线的x得值，应该比array_size少1
	I32 option = 0x1000;//ptp运动模式控制
};

#endif // PASVCONTRL_H
