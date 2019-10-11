#include "passive_control.h"
#include <windows.h>
#include <process.h>
#include <iostream>
#include <fstream>
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
using namespace std;

#define TIMER_SLEEP 0.1
const double Comfort_Pos[2] = {0, 0}; //开始运动的初始位置，人觉得舒服的位置
bool is_exit_thread_ = false;
int loop_counter_in_thread = 0;

PassiveControl::PassiveControl()
{
    //初始化动作队列
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            hermite_time_interval_[i][j] = 0;
            //更新插值位置范围
            hermite_pos_interval_[i][j] = 0;
            //更新插值速度范围
            hermite_vel_interval_[i][j] = 0;
        }
    }
    is_busy_ = false;
    in_record_status_ = false;
    in_move_status_ = false;
    is_teach = false;
}

void PassiveControl::SetActiveControl(ActiveControl *p) { active_control_ = p; }

PassiveControl::~PassiveControl() {}

void PassiveControl::StoreCurrentRecord() { movement_set_.push_back(record_data_); }

// 这里示教和被动运动都写在了这个函数里面,通过flag决定运行哪一部分
unsigned int __stdcall   RecordOrMoveThread(PVOID pParam)
{
    PassiveControl *passive = (PassiveControl *)pParam;
    UINT start, end;
    start = GetTickCount();
    while (TRUE) {
        //延时 TIMER_SLEEP s not figure out how to do ???
        while (TRUE) {
            end = GetTickCount(); // return ms
            if (end - start >= TIMER_SLEEP * 1000) {
                start = end;
                break;
            }
            else {
                SwitchToThread();
            }
        }

        if (is_exit_thread_) {
            break;
        }

        // 是否录制动作， 1s录制一次
        if (passive->in_record_status_ && (loop_counter_in_thread % 10 == 0)) {
            passive->RecordStep();
        }

        //是否开始运动
        if (passive->in_move_status_) {
            passive->MoveStep();
            passive->SampleStep();
        }

        loop_counter_in_thread++;
    }
    if (passive->is_teach) {
        // passive->TeachPosData();
        passive->CruveSmoothing();
    }

    // passive->InterpolationTraceExport();
    // passive->PracticalTraceExport();

    passive->is_busy_ = false;
    return 0;
}

void PassiveControl::ClearMovementSet() { movement_set_.clear(); }

void PassiveControl::StoreMovement(const PassiveData &movement) { movement_set_.push_back(movement); }

bool PassiveControl::IsBusy() { return is_busy_; }

void PassiveControl::BeginMove(int index)
{
    if (index >= movement_set_.size() || index < 0) return;
    // check if in busy(now is in motion)
    if (is_busy_) return;
    move_data_ = movement_set_.at(index); // get passive motion by index
    //各项计数归零
    loop_counter_in_thread = 0;
    hermite_time_counter_ = 0;
    hermite_target_counter_ = 0;

    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            hermite_time_interval_[i][j] = 0;
            //更新插值位置范围
            hermite_pos_interval_[i][j] = 0;
            //更新插值速度范围
            hermite_vel_interval_[i][j] = 0;
        }
    }

    //打开电机，离合器
    ControlCard::GetInstance().SetMotor(ControlCard::MotorOn);
    ControlCard::GetInstance().SetClutch(ControlCard::ClutchOn);

    //关闭示教采集功能
    in_record_status_ = false;
    //打开线程
    is_exit_thread_ = false;
    //开始运动
    in_move_status_ = true;

    HANDLE handle;
    handle = (HANDLE)_beginthreadex(NULL, 0, RecordOrMoveThread, this, 0, NULL);
}
void PassiveControl::StopMove()
{
    //关闭电机
    ControlCard::GetInstance().SetMotor(ControlCard::MotorOff);
    //关闭离合器
    ControlCard::GetInstance().SetClutch(ControlCard::ClutchOff);
    //关闭线程
    in_move_status_ = false;
    is_exit_thread_ = true;
    in_record_status_ = false;
    is_busy_ = false;
}

void PassiveControl::GetCurrentMove(PassiveData &move) { move = move_data_; }

double PassiveControl::PHermite(double foretime[2], double forepos[2], double forevel[2], double t)
{
    double Houtput = 0;
    double a[2] = {0};
    double b[2] = {0};
    a[0] = (1 - 2 * (t - foretime[0]) / (foretime[0] - foretime[1])) * pow((t - foretime[1]) / (foretime[0] - foretime[1]), 2);
    a[1] = (1 - 2 * (t - foretime[1]) / (foretime[1] - foretime[0])) * pow((t - foretime[0]) / (foretime[1] - foretime[0]), 2);
    b[0] = (t - foretime[0]) * pow((t - foretime[1]) / (foretime[0] - foretime[1]), 2);
    b[1] = (t - foretime[1]) * pow((t - foretime[0]) / (foretime[1] - foretime[0]), 2);
    Houtput = a[0] * forepos[0] + a[1] * forepos[1] + b[0] * forevel[0] + b[1] * forevel[1];
    return Houtput;
}
void PassiveControl::BeginRecord()
{
    if (is_busy_) {
        return;
    }
    active_control_->StartMove(999); //被动运动示教id 设置为定值999
    in_record_status_ = true;
    is_exit_thread_ = false;
    is_teach = true;

    // 清空record_data_
    for (int k = 0; k < 2; k++) {
        if (!record_data_.target_positions[k].empty()) {
            record_data_.target_positions[k].clear();
        }

        if (!record_data_.target_velocitys[k].empty()) {
            record_data_.target_velocitys->clear();
        }
    }

    HANDLE handle;
    handle = (HANDLE)_beginthreadex(NULL, 0, RecordOrMoveThread, this, 0, NULL);
}

void PassiveControl::StopRecord()
{
    active_control_->StopMove();
    in_record_status_ = false;
    is_exit_thread_ = true;
    is_busy_ = false;
}

void PassiveControl::GetCurrentRecord(PassiveData &record) { record = record_data_; }


void PassiveControl::RecordStep()
{
    //取角度
    double joint_angle[2]{0};
    double joint_vel[2]{0};
    ControlCard::GetInstance().GetEncoderData(joint_angle);
    ControlCard::GetInstance().GetJointVelocity(joint_vel);
    for (int i = 0; i < 2; i++) {
        record_data_.target_positions[i].push_back(joint_angle[i]);
        record_data_.target_velocitys[i].push_back(joint_vel[i]);
    }
    is_busy_ = true;
}

//这个函数是对目标点进行插值，然后根据插值的位置进行运动
void PassiveControl::MoveStep()
{
    double time = loop_counter_in_thread * 0.1;
    I32 Axis[2] = {ControlCard::ShoulderAxisId, ControlCard::ElbowAxisId};

    //每过一秒就更新插值区间
    if (loop_counter_in_thread % 10 == 0) {
        if (hermite_target_counter_ + 1 < move_data_.target_velocitys[0].size()) {
            for (int i = 0; i < 2; i++) {
                //更新插值时间范围
                hermite_time_interval_[i][0] = time;
                hermite_time_interval_[i][1] = time + 1;
                //更新插值位置范围
                hermite_pos_interval_[i][0] = move_data_.target_positions[i].at(hermite_target_counter_);
                hermite_pos_interval_[i][1] = move_data_.target_positions[i].at(hermite_target_counter_ + 1);
                //更新插值速度范围
                hermite_vel_interval_[i][0] = move_data_.target_velocitys[i].at(hermite_target_counter_);
                hermite_vel_interval_[i][1] = move_data_.target_velocitys[i].at(hermite_target_counter_ + 1);
            }
            hermite_target_counter_++;
        }
        else {
            //运动完成，停止运动
            StopMove();
        }
    }

    //在插值区间内，相当于每100ms就运动到一个新的插值点。
    for (int j = 0; j < 2; j++) {
        double pos = PHermite(hermite_time_interval_[j], hermite_pos_interval_[j], hermite_vel_interval_[j], time);
        sample_data_.interpolation_positions[j].push_back(pos);
        // APS_absolute_move(Axis[j], pos / ControlCard::Unit_Convert, 15 / ControlCard::Unit_Convert);
        APS_ptp_v(Axis[j], option, pos / ControlCard::Unit_Convert, 15 / ControlCard::Unit_Convert, NULL);
    }
    is_busy_ = true;
}

void PassiveControl::SampleStep()
{
    double joint_angle[2]{0};
    ControlCard::GetInstance().GetEncoderData(joint_angle);
    for (int i = 0; i < 2; i++) {
        sample_data_.target_positions[i].push_back(joint_angle[i]);
    }
}

void PassiveControl::SetHWND(HWND hWnd) { hWnd_ = hWnd; }

void PassiveControl::CruveSmoothing()
{
    for (int i = 0; i < 2; ++i) {
        max_pos[i] = GetMaxData(record_data_.target_positions[i]);
    }
    array_size = record_data_.target_positions[0].size();
    if (array_size % 2 == 0) {
        array_size++;
    }
    curve_x = array_size - 1;
    DrawSincruve();
    is_teach = false;
}

double PassiveControl::GetMaxData(std::vector<double> &data)
{
    double maxdata = data[0];
    for (int i = 1; i < data.size(); ++i) {
        if (abs(maxdata) < abs(data[i])) {
            maxdata = data[i];
        }
    }
    return maxdata;
}

void PassiveControl::DrawSincruve()
{
    double curve_pi = M_PI / curve_x;
    for (int j = 0; j < 2; ++j) {
        for (int i = 0; i < array_size; ++i) {
            record_data_.target_positions[j][i] = sin(i * curve_pi) * max_pos[j];
            record_data_.target_velocitys[j][i] = cos(i * curve_pi) * max_pos[j] * curve_pi;
        }
        for (int i = 0; i < 2; ++i) {
            record_data_.target_positions[i][array_size - 1] = 0;
        }
    }
}

void PassiveControl::TeachPosData()
{
    ofstream dataFile1;
    dataFile1.open("teach_position_data.txt", ofstream::app);
    dataFile1 << "shoulder"
              << "   "
              << "elbow" << endl;
    for (int i = 0; i < record_data_.target_positions[0].size(); ++i) {
        dataFile1 << record_data_.target_positions[0][i] << "        " << record_data_.target_positions[1][i] << endl;
    }
    dataFile1.close();
}

void PassiveControl::InterpolationTraceExport()
{
    ofstream dataFile2;
    dataFile2.open("interpolation_trace_data.txt", ofstream::app);
    dataFile2 << "shoulder"
              << "   "
              << "elbow" << endl;
    for (int i = 0; i < sample_data_.interpolation_positions[0].size(); ++i) {
        dataFile2 << sample_data_.interpolation_positions[0][i] << "        " << sample_data_.interpolation_positions[1][i] << endl;
    }
    dataFile2.close();
}

void PassiveControl::PracticalTraceExport()
{
    ofstream dataFile3;
    dataFile3.open("practice_trace_data.txt", ofstream::app);
    dataFile3 << "shoulder"
              << "   "
              << "elbow" << endl;
    for (int i = 0; i < sample_data_.target_positions[0].size(); ++i) {
        dataFile3 << sample_data_.target_positions[0][i] << "        " << sample_data_.target_positions[1][i] << endl;
    }
    dataFile3.close();
}