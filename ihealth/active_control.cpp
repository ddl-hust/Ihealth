﻿#include "active_control.h"

#include <iostream>
#include <sstream>
#include <ctime>
#include <fstream>
#include <process.h>
#include <windows.h>

#include "Matrix.h"
#include "Log.h"
#include "data_acquisition.h"
#include "pupiltracker/utils.h"

// spdlog debug
#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>
// #include "LoggerMacros.h"
using namespace Eigen;
using namespace std;

double Force_Fc = 0.3;
double Force_a = 0.3;
double Force_b = 1;

vector<double> torque_data[2];

double anglearm = 0; //手臂关节角
double angleshoul = 0; //肩部关节角
double Ud_Arm = 0; //力控模式算出手臂的命令速度
double Ud_Shoul = 0; //力控模式算出肩部的命令速度
const char *FCH = "Dev2/ai6"; //握力采集通道

static const int kPlaneMaxX = 734;
static const int kPlaneMaxY = 601;

static const int kRagMaxX = 710;
static const int kRagMaxY = 596;

double ActiveControl::six_dimforce[6]{0};

extern Vector3d AxisDirection[5]{
    Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0),
};
extern Vector3d AxisPosition[5]{
    Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector3d(0, 0, 0),
};
Matrix3d RF13;
Matrix3d sixdim_rotation;
//导出病人数据将主动运动与病人id关联起来
struct ExportPaitentData
{
    int paitent_id;
    ActiveControl* active_control; 
};
ActiveControl::ActiveControl()
{
    move_thread_handle_ = 0;
    is_exit_thread_ = false;
    is_moving_ = false;
    LoadParamFromFile();
}

void ActiveControl::LoadParamFromFile()
{
    pupiltracker::ConfigFile cfg;
    cfg.read("../../resource/params/active_control.cfg");
    string s;
    stringstream ss;
    string item;
    // RF13  这部分注意配置文件
    s = cfg.get<string>("RF13");
    ss.clear();
    ss.str(s);
    vector<double> rf13;
    while (getline(ss, item, ',')) {
        rf13.push_back(stod(item));
    }
    RF13 << rf13[0], rf13[1], rf13[2], rf13[3], rf13[4], rf13[5], rf13[6], rf13[7], rf13[8];

    // sixdim_rotation
    s = cfg.get<string>("sixdim_rotation");
    ss.clear();
    ss.str(s);
    vector<int> r;
    while (getline(ss, item, ',')) {
        r.push_back(stoi(item));
    }
    sixdim_rotation << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];

    // rotate_matrix_
    // s = cfg.get<string>("rotate_matrix");
    // ss.clear();
    // ss.str(s);
    // vector<int> r;
    // while (getline(ss, item, ',')) {
    //     r.push_back(stoi(item));
    // }
    // rotate_matrix_ << r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7], r[8];

    // force_position_
    s = cfg.get<string>("force_position");
    ss.clear();
    ss.str(s);
    vector<double> f;
    while (getline(ss, item, ',')) {
        f.push_back(stod(item));
    }
    force_position_ << f[0], f[1], f[2];

    // param depend on left or right arm
    int is_left = cfg.get<int>("is_left");
    if (is_left) {
        AxisDirection[0] = Vector3d(1.0, 0, 0);
        AxisDirection[1] = Vector3d(0, 0, 1.0);
        AxisDirection[2] = Vector3d(0, -1.0, 0);
        AxisDirection[3] = Vector3d(0, -1.0, 0);
        AxisDirection[4] = Vector3d(-1.0, 0, 0);
        // axis origin coexist
        AxisPosition[0] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
        AxisPosition[1] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
        AxisPosition[2] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
        AxisPosition[3] = Vector3d(-LowerArmLength, 0, 0);
        AxisPosition[4] = Vector3d(-LowerArmLength, 0, 0);
    }
    else {
        AxisDirection[0] = Vector3d(-1, 0, 0);
        AxisDirection[1] = Vector3d(0, 0, -1);
        AxisDirection[2] = Vector3d(0, -1, 0);
        AxisDirection[3] = Vector3d(0, -1, 0);
        AxisDirection[4] = Vector3d(1, 0, 0);

        AxisPosition[0] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
        AxisPosition[1] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
        AxisPosition[2] = Vector3d(-UpperArmLength - LowerArmLength, 0, 0);
        AxisPosition[3] = Vector3d(-LowerArmLength, 0, 0);
        AxisPosition[4] = Vector3d(-LowerArmLength, 0, 0);
    }

    // cycle_time_in_second
    cycle_time_in_second_ = cfg.get<double>("cycle_time_in_second");

    // shoulder and elbow range
    shoulder_angle_max_ = cfg.get<double>("shoulder_angle_max");
    elbow_angle_max_ = cfg.get<double>("elbow_angle_max");
    elbow_Sensitivity_ = 3; // default sensitivity
    // AllocConsole();
    // freopen("CONOUT$", "w", stdout);
    // cout << rotate_matrix_ << endl;
    // cout << force_position_ << endl;
    // cout << is_left << endl;
    // cout << AxisPosition << endl;
    // cout << AxisDirection << endl;
}

ActiveControl::~ActiveControl()
{
    // DataAcquisition::GetInstance().StopTask();
}

unsigned int __stdcall ActiveMoveThread(PVOID pParam) // what is that meaning  whne actually call there is not parm
{   
    ExportPaitentData *export_data= (ExportPaitentData*)pParam;
    ActiveControl *active = export_data->active_control;
    int paitent_id=export_data->paitent_id;
    UINT start, end;
    start = GetTickCount();
    double torque_sum[6]{0.0};
    double torque_buf[6]{0.0};
    // use 10 times torque data smooth
    for (int i = 0; i < 10; ++i) {
        DataAcquisition::GetInstance().AcquisiteSixDemensionData(torque_buf);
        for (int j = 0; j < 6; ++j) {
            torque_sum[j] += torque_buf[j];
        }
    }
    for (int i = 0; i < 6; ++i) {
        active->six_dimension_offset_[i] = torque_sum[i] / 10;
    }
    // use 10 times pressure data smooth
    double pressure_buf[2]{0.0};
    double pressure_sum[2]{0.0};
    for (int i = 0; i < 10; ++i) {
        DataAcquisition::GetInstance().AcquisiteTensionData(pressure_buf);
        for (int j = 0; j < 2; ++j) {
            pressure_sum[j] += pressure_buf[j];
        }
    }
    for (int i = 0; i < 2; ++i) {
        active->elbow_offset_[i] = pressure_sum[i] / 10;
    }
    DataAcquisition::GetInstance().StopSixDemTask();
    DataAcquisition::GetInstance().StartSixDemTask();
    // all the output sensor data
    std::string pathname="..\\..\\resource\\ExportData\\";
    time_t t = time(0);
    char ch[64];
    strftime(ch, sizeof(ch), "%Y-%m-%d %H-%M-%S", localtime(&t)); //年-月-日 时-分-秒
    std::string paitent_info="patient_"+std::to_string(paitent_id)+"_";
    ofstream joint_value(pathname+paitent_info+"joint_"+ch+".txt", ios::app | ios::out);
    ofstream torque_value(pathname+paitent_info+"torque_"+ch+".txt", ios::app | ios::out);
    ofstream sixdim_force_value(pathname+paitent_info+"sixdim_force_"+ch+".txt", ios::app | ios::out);
    ofstream pressure_force_value(pathname+paitent_info+"pressure_force_"+ch+".txt", ios::app | ios::out);
    ofstream sum_pressure_force_value(pathname+paitent_info+"sum_pressure_force_"+ch+".txt", ios::app | ios::out);
    double angle[2]{0};
    double torque[2]{0};
    double elbow_pressure[2]{0};
    double six_dim_force[6]{0};

    spdlog::info("ready to write joint data to txt");
    joint_value << " shoulder/degree(-360-360)  "
                << "  elbow/degree(-360-360)  " << endl;
    torque_value << " shoulder(N.m)  "
                << "  elbow(N.m)  " << endl;
    sixdim_force_value<<" fx(N) "<<" fy(N) "<<" fz(N) "<<" tx(N.m) "<<" ty(N.m) "<<" tz(N.m) "<<endl;
    sum_pressure_force_value<<"F>0表示肘曲"<<"F<0表示肘伸"<<endl;
    


    while (true) {
        if (active->is_exit_thread_) {
            break; 
        }
        // 每隔一定时间进行一次循环，这个循环时间应该是可调的。
        while (true) {
            end = GetTickCount();
            if (end - start >= active->cycle_time_in_second_ * 1000) { // what is that meaning ???
                start = end;
                break;
            }
            else {
                SwitchToThread(); // so switch to which thread???
            }
        }
        // Ih the active cointrol thread we open sixdimforce ,pressure thread to get sixdimforce and pressure data
        active->Step();
        spdlog::info("{} {} {} ready to into pressure thread", __LINE__, __FILE__, __FUNCTION__);
        active->PressureStep();
        // To-Do
        /**
         * export joint value ,torque ,pressure ,and so on
         * I think for doctor do experienment maybe they need all sensor data
         * So in case ,we can export all sensor data
         */
        // std::vector<std::vector<double>> joint_data;
        ControlCard::GetInstance().GetEncoderData(angle);
        DataAcquisition::GetInstance().AcquisiteTorqueData(torque);
        DataAcquisition::GetInstance().AcquisiteTensionData(elbow_pressure);
        DataAcquisition::GetInstance().AcquisiteSixDemensionData(six_dim_force);
        joint_value << angle[0] << "          " << angle[1] << std::endl;
        torque_value << torque[0] << "          " <<torque[1] << std::endl;
        sixdim_force_value << six_dim_force[0] << "          " << six_dim_force[1] << "          " << six_dim_force[2] << "          " << six_dim_force[3]
                           << "          " << six_dim_force[4] << "          " << six_dim_force[5] << std::endl;
        // pressure_force_value << elbow_pressure[0] * 10<< "          " << elbow_pressure[1] * 10 << std::endl;
        sum_pressure_force_value<< elbow_pressure[0] * 10- elbow_pressure[1] * 10<<endl;
    }
    spdlog::info("The end of active eith presure thread");
    time_t now = time(0);
   // 把 now 转换为字符串形式
   char* dt = ctime(&now);
    // joint_value<<"end of one train current time is : "<<dt<<endl;
    joint_value.close();
    torque_value.close();
    sixdim_force_value.close();
    pressure_force_value.close();
    sum_pressure_force_value.close();
    return 0;
}
/******************只有六维力线程******************/
unsigned int __stdcall ActiveMoveThreadOnlySixDim(PVOID pParam)
{
    ExportPaitentData *export_data= (ExportPaitentData*)pParam;
    ActiveControl *active = export_data->active_control;
    int paitent_id=export_data->paitent_id;
    UINT start, end;
    start = GetTickCount();
    // 求六维力传感器的偏置
    double sum[6]{0.0};
    double buf[6]{0.0};
    for (int i = 0; i < 10; ++i) {
        DataAcquisition::GetInstance().AcquisiteSixDemensionData(buf);
        for (int j = 0; j < 6; ++j) {
            sum[j] += buf[j];
        }
    }
    for (int i = 0; i < 6; ++i) {
        active->six_dimension_offset_[i] = sum[i] / 10;
    }

    DataAcquisition::GetInstance().StopSixDemTask();
    DataAcquisition::GetInstance().StartSixDemTask();
	// all the output sensor data
    std::string pathname="..\\..\\resource\\ExportData\\";
    time_t t = time(0);
    char ch[64];
    strftime(ch, sizeof(ch), "%Y-%m-%d %H-%M-%S", localtime(&t)); //年-月-日 时-分-秒
    std::string paitent_info="patient_"+std::to_string(paitent_id)+"_";
    ofstream joint_value(pathname+paitent_info+"joint_"+ch+".txt", ios::app | ios::out);
    ofstream torque_value(pathname+paitent_info+"torque_"+ch+".txt", ios::app | ios::out);
    ofstream sixdim_force_value(pathname+paitent_info+"sixdim_force_"+ch+".txt", ios::app | ios::out);
    ofstream pressure_force_value(pathname+paitent_info+"pressure_force_"+ch+".txt", ios::app | ios::out);
    ofstream sum_pressure_force_value(pathname+paitent_info+"sum_pressure_force_"+ch+".txt", ios::app | ios::out);
    double angle[2]{0};
    double torque[2]{0};
    double elbow_pressure[2]{0};
    double six_dim_force[6]{0};

    spdlog::info("ready to write joint data to txt");
    joint_value << " shoulder/degree(-360-360)  "
                << "  elbow/degree(-360-360)  " << endl;
    torque_value << " shoulder(N.m)  "
                << "  elbow(N.m)  " << endl;
    sixdim_force_value<<" fx(N) "<<" fy(N) "<<" fz(N) "<<" tx(N.m) "<<" ty(N.m) "<<" tz(N.m) "<<endl;
    sum_pressure_force_value<<"F>0表示肘曲"<<"F<0表示肘伸"<<endl;

    while (true) {
        if (active->is_exit_thread_) {
            break;
        }

        // 每隔一定时间进行一次循环，这个循环时间应该是可调的。
        while (true) {
            end = GetTickCount();
            if (end - start >= active->cycle_time_in_second_ * 1000) {
                start = end;
                break;
            }
            else {
                SwitchToThread();
            }
        }
        //六维力线程
        active->SixDimForceStep();
		ControlCard::GetInstance().GetEncoderData(angle);
        DataAcquisition::GetInstance().AcquisiteTorqueData(torque);
        DataAcquisition::GetInstance().AcquisiteTensionData(elbow_pressure);
        DataAcquisition::GetInstance().AcquisiteSixDemensionData(six_dim_force);
        joint_value << angle[0] << "          " << angle[1] << std::endl;
        torque_value << torque[0] << "          " <<torque[1] << std::endl;
        sixdim_force_value << six_dim_force[0] << "          " << six_dim_force[1] << "          " << six_dim_force[2] << "          " << six_dim_force[3]
                           << "          " << six_dim_force[4] << "          " << six_dim_force[5] << std::endl;
        // pressure_force_value << elbow_pressure[0] * 10<< "          " << elbow_pressure[1] * 10 << std::endl;
        sum_pressure_force_value<< elbow_pressure[0] * 10- elbow_pressure[1] * 10<<endl;
    }
	spdlog::info("The end of active thread oinly with six dim force");
    joint_value.close();
    torque_value.close();
    sixdim_force_value.close();
    pressure_force_value.close();
    sum_pressure_force_value.close();
    return 0;
}



void ActiveControl::MoveInNewThread(int id)
{   
    ExportPaitentData* export_data=new ExportPaitentData();
    export_data->active_control=this;
    export_data->paitent_id=id;
    is_exit_thread_ = false;
    // move_thread_handle_ = (HANDLE)_beginthreadex(NULL, 0, ActiveMoveThread, export_data, 0, NULL);
    if (m_pressure_sensor_enable) {
        move_thread_handle_ = (HANDLE)_beginthreadex(NULL, 0, ActiveMoveThread, export_data, 0, NULL);
    }
    else {
        move_thread_handle_ = (HANDLE)_beginthreadex(NULL, 0,ActiveMoveThreadOnlySixDim, export_data, 0, NULL);
    }
}


void ActiveControl::ExitMoveThread()
{
    is_exit_thread_ = true;

    if (move_thread_handle_ != 0) {
        ::WaitForSingleObject(move_thread_handle_, INFINITE);
        move_thread_handle_ = 0;
    }
}

void ActiveControl::StartMove(int id)
{ // set motor, cluth on and open a new thread
    ControlCard::GetInstance().SetMotor(ControlCard::MotorOn);
    ControlCard::GetInstance().SetClutch(ControlCard::ClutchOn);
    is_moving_ = true;
    MoveInNewThread(id);
}

void ActiveControl::StopMove()
{
    //这里不放开离合的原因是为了防止中间位置松开离合导致手臂迅速下坠
    ControlCard::GetInstance().SetMotor(ControlCard::MotorOff);
    is_moving_ = false;
    ExitMoveThread();
}

void ActiveControl::Step()
{
    double readings[6] = {0}; // six_dim_force  data read from sensor
    double distData[6] = {0}; // read data transform to handle(dist) frame
    double filtedData[6] = {0};
    double bias[6] = {0};
    double sub_bias[6] = {0};

    DataAcquisition::GetInstance().AcquisiteSixDemensionData(readings);

    // what is  ???
    torque_data[0].push_back(detect.shoulder_torque);
    torque_data[1].push_back(detect.elbow_torque);

    /** six_force minus offet ,according to our actually experiement,the 3th,6th's force direction
     * should be reverse (maybe it's the XXX)
     */
    for (int i = 0; i < 6; ++i) {
        sub_bias[i] = readings[i] - six_dimension_offset_[i];
    }
    sub_bias[2] = -sub_bias[2];
    sub_bias[5] = -sub_bias[5];
    Trans2Filter(sub_bias, filtedData);
    for (int i = 0; i < 6; ++i) {
        six_dimforce[i] = filtedData[i];
    }
}
void ActiveControl::SixDimForceStep()
{
    double readings[6] = {0};
    double distData[6] = {0};
    double filtedData[6] = {0};
    double bias[6] = {0};
    double sub_bias[6] = {0};
    double force_vector = 0;
    double vel[2] = {0};

    DataAcquisition::GetInstance().AcquisiteSixDemensionData(readings);

    torque_data[0].push_back(detect.shoulder_torque);
    torque_data[1].push_back(detect.elbow_torque);

    // 求减去偏置之后的六维力，这里对z轴的力和力矩做了一个反向
    for (int i = 0; i < 6; ++i) {
        sub_bias[i] = readings[i] - six_dimension_offset_[i];
    }
    sub_bias[2] = -sub_bias[2];
    sub_bias[5] = -sub_bias[5];
    Trans2Filter(sub_bias, filtedData);
    SixDimForceMomentCalculation(filtedData, vel);
    Ud_Shoul = shoulder_Sensitivity_ * vel[0];
    Ud_Arm = shoulder_Sensitivity_ * vel[1];
    if (Ud_Arm > 5) {
        Ud_Arm = 5;
    }
    else if (Ud_Arm < -5) {
        Ud_Arm = -5;
    }
    if (Ud_Shoul > 5) {
        Ud_Shoul = 5;
    }
    else if (Ud_Shoul < -5) {
        Ud_Shoul = -5;
    }

    if (is_moving_) {
        ActMove();
    }
}
void ActiveControl::SixDimForceMomentCalculation(double ForceVector[6], double vel[2])
{
    VectorXd six_dimensional_force_simulation(6);
    VectorXd v_moment(5);
    Vector2d v_vel;
    MatrixXd pos(2, 1);

    double angle[2];
    double moment[3];

    ControlCard::GetInstance().GetEncoderData(angle);

    pos(0, 0) = angle[0];
    pos(1, 0) = angle[1];
    joint_angle[0] = angle[0];
    joint_angle[1] = angle[1];

    six_dimensional_force_simulation(0) = ForceVector[0];
    six_dimensional_force_simulation(1) = ForceVector[1];
    six_dimensional_force_simulation(2) = ForceVector[2];
    six_dimensional_force_simulation(3) = ForceVector[3];
    six_dimensional_force_simulation(4) = ForceVector[4];
    six_dimensional_force_simulation(5) = ForceVector[5];

    MomentBalance(six_dimensional_force_simulation, angle, moment);

    for (int i = 0; i < 3; ++i) {
        v_moment(i) = moment[i];
    }
    v_moment(3) = 1.12 * v_moment(2);
    v_moment(4) = 1.48 * v_moment(2);

    AdmittanceControl(v_moment, v_vel);

    // AllocConsole();
    // freopen("CONOUT$", "w", stdout);
    // printf("v_moment1:%lf    v_moment2:%lf    v_moment3:%lf     v_moment4:%lf    v_moment5:%lf \n", v_moment(0), v_moment(1), v_moment(2), v_moment(3),
    // v_moment(4)); printf("v_vel1: %lf     v_vel2: %lf\n", v_vel(0), v_vel(1));

    vel[0] = v_vel(0);
    vel[1] = v_vel(1);
}
/******************************for we change the control method no need torque***************************************/
// void ActiveControl::TorqueStep()
// {
//     //力矩传感器相关
//     double torque_data[2]{0};
//     double torque_subdata[2]{0};
//     double torque_alldata[5]{0};

//     Vector2d vel;
//     VectorXd torque(5);

//     DataAcquisition::GetInstance().AcquisiteTorqueData();

//     //减偏置,0是肘，1是肩
//     // for (int i = 0; i < 2; ++i) {
//     //	torque_subdata[i] = torque_data[i] - torque_offset_[i];
//     //}
//     torque_subdata[0] = DataAcquisition::GetInstance().torque_data[5];
//     torque_subdata[1] = DataAcquisition::GetInstance().torque_data[15];

//     ActiveTorqueToAllTorque(torque_subdata, torque_alldata);

//     for (int i = 0; i < 5; ++i) {
//         torque(i) = torque_alldata[i]; // just translate array to vector
//     }

//     AdmittanceControl(torque, vel);

//     Ud_Shoul = vel(0);
//     Ud_Arm = vel(1);

//     if ((Ud_Arm > -0.5) && (Ud_Arm < 0.5)) {
//         Ud_Arm = 0;
//     }
//     if ((Ud_Shoul > -0.5) && (Ud_Shoul < 0.5)) {
//         Ud_Shoul = 0;
//     }
//     if (Ud_Arm > 3) {
//         Ud_Arm = 3;
//     }
//     else if (Ud_Arm < -3) {
//         Ud_Arm = -3;
//     }
//     if (Ud_Shoul > 3) {
//         Ud_Shoul = 3;
//     }
//     else if (Ud_Shoul < -3) {
//         Ud_Shoul = -3;
//     }

//     if (is_moving_) {
//         ActMove();
//     }
// }

void ActiveControl::PressureStep()
{
    double elbow_suboffset[2] = {0};
    double elbow_smooth[4] = {0};
    double elbow_pressure_data[2] = {0};
    double force_vector = 0;
    double vel = 0;

    DataAcquisition::GetInstance().AcquisiteTensionData(elbow_pressure_data);


    //减偏置
    for (int i = 0; i < 2; ++i) {
        elbow_suboffset[i] = elbow_pressure_data[i] - elbow_offset_[i];
    }
    // convert unit from voltage to force
    for (int j = 0; j < 2; ++j) {
        elbow_suboffset[j] = 10 * elbow_suboffset[j];
    }

    //进行二阶巴特沃兹滤波
    Trans2FilterForPressure(elbow_suboffset, elbow_smooth);

    //将传感器数据转成力矢量
    // SensorDataToForceVector(shoulder_suboffset, elbow_suboffset, force_vector);
    force_vector = elbow_smooth[0] - elbow_smooth[1];

    MomentCalculation(force_vector, vel); //???
    // open console to show debug info
    AllocConsole();
    freopen("CONOUT$", "w", stdout);
    spdlog::info("{} the elbow sensitivity :{} ,the shoulder sensitity :{} ", __LINE__, elbow_Sensitivity_, shoulder_Sensitivity_);
    if (joint_angle[0] < 10) {
        Ud_Shoul = -3 * six_dimforce[0];
    }
    else {
        // if (Ud_Shoul > 0) {
        Ud_Shoul = shoulder_Sensitivity_ * vel;
        //}
        // else {
        //	Ud_Shoul = 3 * vel;
        //}
    }
    if (Ud_Arm > 0) {
        // Ud_Arm = 3 * force_vector;
        Ud_Arm = elbow_Sensitivity_ * force_vector;
    }
    else {
        Ud_Arm = 2 * force_vector;
    }

    //这里遇到了速度输入为0时会被忽略掉的问题，所以先把这里注释掉
    // if ((Ud_Arm > -0.5) && (Ud_Arm < 0.5)) {
    //	Ud_Arm = 0;
    //}
    // if ((Ud_Shoul > -0.5) && (Ud_Shoul < 0.5)) {
    //	Ud_Shoul = 0;
    //}
    if (Ud_Arm > 4) {
        Ud_Arm = 4;
    }
    else if (Ud_Arm < -4) {
        Ud_Arm = -4;
    }
    if (Ud_Shoul > 5) {
        Ud_Shoul = 5;
    }
    else if (Ud_Shoul < -5) {
        Ud_Shoul = -5;
    }

    // AllocConsole();
    // freopen("CONOUT$", "w", stdout);
    // cout << "Ud_Shoul:" << Ud_Shoul << "     " << "Ud_Arm:" << Ud_Arm << "\n" << endl;

    if (is_moving_) {
        ActMove();
    }

    // Sleep(100);  
}

// void ActiveControl::Raw2Trans(double RAWData[6], double DistData[6])
// {
//     //这一段就是为了把力从六维力传感器上传到手柄上，这里的A就是总的一个转换矩阵。
//     //具体的旋转矩阵我们要根据六维力的安装确定坐标系方向之后然后再确定。
//     MatrixXd A(6, 6);
//     A.setZero();
//     VectorXd Value_Origi(6);
//     VectorXd Value_Convers(6);
//     Matrix3d ForcePositionHat;
//     //这里就是这个p，我们可以想象，fx不会产生x方向的力矩，fy产生的看z坐标，fz产生的y坐标。
//     //这里做的就是把力矩弄过去。这个相对坐标都是六维力坐标在手柄坐标系下的位置。
//     //比如fx在y方向上有一个力臂，就会产生一个z方向上的力矩。这个力矩的方向和相对位置无关。
//     //所以这个地方我们不用改这个ForcePositionHat，只用改force_position_这个相对位置就可以了
//     ForcePositionHat << 0, -force_position_[2], force_position_[1], force_position_[2], 0, -force_position_[0], -force_position_[1], force_position_[0], 0;
//     A.block(0, 0, 3, 3) = rotate_matrix_;
//     A.block(0, 3, 3, 3) = ForcePositionHat * rotate_matrix_;
//     A.block(3, 3, 3, 3) = rotate_matrix_;


//     //之前是fxfyfzMxMyMz,现在变成MxMyMzfxfyfz
//     for (int i = 0; i < 6; i++) {
//         if (i < 3) {
//             Value_Origi(i) = RAWData[i + 3];
//         }
//         else {
//             Value_Origi(i) = RAWData[i - 3];
//         }
//     }

//     //这里计算后就是
//     Value_Convers = A * Value_Origi;
//     for (int m = 0; m < 6; m++) {
//         DistData[m] = Value_Convers(m);
//     }
// }

void ActiveControl::Trans2Filter(double TransData[6], double FiltedData[6])
{
    double Wc = 5;
    double Ts = 0.1;
    static int i = 0;
    static double Last_Buffer[6] = {0};
    static double Last2_Buffer[6] = {0};
    static double Force_Buffer[6] = {0};
    static double Last_FT[6] = {0};
    static double Last2_FT[6] = {0};
    for (int m = 0; m < 6; m++) {
        if (i == 0) {
            Last2_Buffer[m] = TransData[m];
            FiltedData[m] = 0;
            i++;
        }
        else if (i == 1) {
            Last_Buffer[m] = TransData[m];
            FiltedData[m] = 0;
            i++;
        }
        else {
            //二阶巴特沃斯低通滤波器
            Force_Buffer[m] = TransData[m];
            FiltedData[m] = (1 / (Wc * Wc + 2 * 1.414 * Wc / Ts + 4 / (Ts * Ts)))
                            * ((Wc * Wc) * Force_Buffer[m] + (2 * Wc * Wc) * Last_Buffer[m] + (Wc * Wc) * Last2_Buffer[m]
                               - (2 * Wc * Wc - 8 / (Ts * Ts)) * Last_FT[m] - (Wc * Wc - 2 * 1.414 * Wc / Ts + 4 / (Ts * Ts)) * Last2_FT[m]);

            Last2_FT[m] = Last_FT[m];
            Last_FT[m] = FiltedData[m];
            Last2_Buffer[m] = Last_Buffer[m];
            Last_Buffer[m] = Force_Buffer[m];
        }
    }
    // printf("fx:%lf    fy:%lf    fz:%lf \n Mx:%lf    My:%lf    Mz:%lf \n", FiltedData[3], FiltedData[4], FiltedData[5], FiltedData[0], FiltedData[1],
    // FiltedData[2]);
}

void ActiveControl::Trans2FilterForPressure(double TransData[2], double FiltedData[2])
{
    double Wc = 3;
    double Ts = 0.1;
    static int i = 0;
    static double Last_Buffer[2] = {0};
    static double Last2_Buffer[2] = {0};
    static double Force_Buffer[2] = {0};
    static double Last_FT[2] = {0};
    static double Last2_FT[2] = {0};
    for (int m = 0; m < 2; m++) {
        if (i == 0) {
            Last2_Buffer[m] = TransData[m];
            FiltedData[m] = 0;
            i++;
        }
        else if (i == 1) {
            Last_Buffer[m] = TransData[m];
            FiltedData[m] = 0;
            i++;
        }
        else {
            //二阶巴特沃斯低通滤波器
            Force_Buffer[m] = TransData[m];
            FiltedData[m] = (1 / (Wc * Wc + 2 * 1.414 * Wc / Ts + 4 / (Ts * Ts)))
                            * ((Wc * Wc) * Force_Buffer[m] + (2 * Wc * Wc) * Last_Buffer[m] + (Wc * Wc) * Last2_Buffer[m]
                               - (2 * Wc * Wc - 8 / (Ts * Ts)) * Last_FT[m] - (Wc * Wc - 2 * 1.414 * Wc / Ts + 4 / (Ts * Ts)) * Last2_FT[m]);

            Last2_FT[m] = Last_FT[m];
            Last_FT[m] = FiltedData[m];
            Last2_Buffer[m] = Last_Buffer[m];
            Last_Buffer[m] = Force_Buffer[m];
        }
    }
}

// void ActiveControl::SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4])
// {
//     double shoulderdataY = shouldersensordata[0] - shouldersensordata[1];
//     double shoulderdataZ = shouldersensordata[2] - shouldersensordata[3];
//     double elbowdataY = elbowsensordata[0] - elbowsensordata[1];
//     //这里因为8在Y+,7在Y-,所以用8-7表示正向
//     double elbowdataZ = elbowsensordata[3] - elbowsensordata[2];

//     //合成的力矢量
//     // Vector2d shoulderforce;
//     // Vector2d elbowforce;
//     // shoulderforce << shoulderdataY, shoulderdataZ;
//     // elbowforce << elbowdataY, elbowdataZ;

//     // AllocConsole();
//     // freopen("CONOUT$", "w", stdout);
//     // cout <<"shoulderforce:\n"<< shoulderforce << "\n" << "elbowforce:\n"<<elbowforce << endl;

//     ForceVector[0] = shoulderdataY;
//     ForceVector[1] = shoulderdataZ;
//     ForceVector[2] = elbowdataY;
//     ForceVector[3] = elbowdataZ;
// }

void ActiveControl::MomentCalculation(double ForceVector, double &vel)
{
    MatrixXd m_vel(2, 1);
    MatrixXd pos(2, 1);

    VectorXd shoulder_force_moment_vector(6);
    VectorXd six_dimensional_force_simulation(6);
    VectorXd v_moment(5);

    double angle[2];
    double moment[3];

    ControlCard::GetInstance().GetEncoderData(angle);

    pos(0, 0) = angle[0];
    pos(1, 0) = angle[1];
    joint_angle[0] = angle[0];
    joint_angle[1] = angle[1];

    six_dimensional_force_simulation(0) = six_dimforce[0];
    six_dimensional_force_simulation(1) = six_dimforce[1];
    six_dimensional_force_simulation(2) = six_dimforce[2];
    six_dimensional_force_simulation(3) = six_dimforce[3];
    six_dimensional_force_simulation(4) = six_dimforce[4];
    six_dimensional_force_simulation(5) = six_dimforce[5];

    MomentBalance(six_dimensional_force_simulation, angle, moment);

    // for (int i = 0; i < 5; ++i) {
    //	v_moment(i) = moment[i];
    //}

    ////关节空间投影到驱动空间
    // AdmittanceControl(v_moment, m_vel);

    // AllocConsole();
    // freopen("CONOUT$", "w", stdout);
    // printf("moment1:%lf      moment2:%lf      moment3:%lf      moment4:%lf      moment5:%lf  \n", moment[0], moment[1], moment[2], moment[3], moment[4]);
    // printf("angle1:%lf     angle2:%lf \n", joint_angle[0], joint_angle[1]);

    vel = moment[0];
}

// void ActiveControl::FiltedVolt2Vel(double FiltedData[6])
// {
//     MatrixXd Vel(2, 1);
//     MatrixXd Pos(2, 1);
//     MatrixXd A(6, 6);
//     VectorXd Six_Sensor_Convert(6);
//     double angle[2];
//     ControlCard::GetInstance().GetEncoderData(angle);
//     Pos(0, 0) = angle[0];
//     Pos(1, 0) = angle[1];

//     // AllocConsole();
//     // freopen("CONOUT$", "w", stdout);
//     // printf("elbow angle: %lf\n", angle[1]);
//     // printf("shoulder angle: %lf\n", angle[0]);
//     // printf("fx:%lf    fy:%lf    fz:%lf \n Mx:%lf    My:%lf    Mz:%lf \n", FiltedData[3], FiltedData[4], FiltedData[5], FiltedData[0], FiltedData[1],
//     // FiltedData[2]);

//     for (int i = 0; i < 6; i++) {
//         Six_Sensor_Convert(i) = FiltedData[i];
//     }
//     damping_control(Six_Sensor_Convert, Pos, Vel, Force_Fc, Force_a, Force_b);
//     Ud_Shoul = Vel(0, 0);
//     Ud_Arm = Vel(1, 0);

//     if ((Ud_Arm > -0.5) && (Ud_Arm < 0.5)) {
//         Ud_Arm = 0;
//     }
//     if ((Ud_Shoul > -0.5) && (Ud_Shoul < 0.5)) {
//         Ud_Shoul = 0;
//     }
//     if (Ud_Arm > 5) {
//         Ud_Arm = 5;
//     }
//     else if (Ud_Arm < -5) {
//         Ud_Arm = -5;
//     }
//     if (Ud_Shoul > 5) {
//         Ud_Shoul = 5;
//     }
//     else if (Ud_Shoul < -5) {
//         Ud_Shoul = -5;
//     }

//     // printf("肩部速度: %lf\n", Ud_Shoul);
//     // printf("肘部速度: %lf\n", Ud_Arm);
// }

void ActiveControl::ActiveTorqueToAllTorque(double torque[2], double alltorque[5])
{
    alltorque[0] = torque[1]; // below coff how to determine
    alltorque[1] = torque[1] * 3 / 2;
    alltorque[2] = torque[0];
    alltorque[3] = torque[0] * 56 / 50;
    alltorque[4] = torque[0] * 74 / 50;
}

void ActiveControl::ActMove()
{
    ControlCard::GetInstance().ProtectedVelocityMove(ControlCard::ShoulderAxisId, Ud_Shoul);
    ControlCard::GetInstance().ProtectedVelocityMove(ControlCard::ElbowAxisId, Ud_Arm);
}

void ActiveControl::CalculatePlaneXY(short rangeX, short rangeY, double XY[2])
{
    // MatrixXd Theta(5, 1);
    // MatrixXd T0h(4, 4);
    // VectorXd Pos(2);

    double angle[2] = {0};
    ControlCard::GetInstance().GetEncoderData(angle);

    int x = (angle[0] / shoulder_angle_max_) * kPlaneMaxX;
    int y = (angle[1] / elbow_angle_max_) * kPlaneMaxY;

    if (y < 0) {
        y = 0;
    }
    else if (y > 100) {
        y = 100;
    }

    if (x < 0) {
        x = 0;
    }
    else if (x > kPlaneMaxX) {
        x = kPlaneMaxX;
    }

    // XY[0] = kPlaneMaxX - x;
    XY[0] = x;
    XY[1] = kPlaneMaxY - y;

    /*Pos << angle[0], angle[1];
    fwd_geo_coup(Pos, Theta);
    fwd_geo_kineB(Theta, T0h);
    double x = -T0h(1, 3);
    double y = ShoulderLength + UpperArmLength + LowerArmLength - T0h(0, 3);

    x = std::max<double>(std::min<double>(x, 0.3), 0);
    y = std::max<double>(std::min<double>(y, 0.3), 0);
    XY[0] = (x / 0.3)*rangeX;
    XY[1] =(1 - 0.3 * y / 0.3)*rangeY;*/
}

void ActiveControl::CalculateRagXY(double XY[2])
{
    double angle[2] = {0};
    ControlCard::GetInstance().GetEncoderData(angle);

    // 根据比例得到抹布位置
    int x = (angle[0] / shoulder_angle_max_) * kRagMaxX;
    int y = (angle[1] / elbow_angle_max_) * kRagMaxY;

    if (y < 0) {
        y = 0;
    }
    else if (y > kRagMaxY) {
        y = kRagMaxY;
    }

    if (x < 0) {
        x = 0;
    }
    else if (x > kRagMaxX) {
        x = kRagMaxX;
    }

    XY[0] = x;
    XY[1] = kRagMaxY - y;
}

void ActiveControl::SetDamping(float FC) { Force_Fc = FC; }
void ActiveControl::SetSAAMax(double saa) { shoulder_angle_max_ = saa; }
void ActiveControl::SetSFEMax(double sfe) { elbow_angle_max_ = sfe; }
void ActiveControl::SetArmSensitivity(double arm_senitivity) { elbow_Sensitivity_ = arm_senitivity; }
void ActiveControl::SetShoulderSensitivity(double shoulder_senitivity) { shoulder_Sensitivity_ = shoulder_senitivity; }


// To-Do 下一步改善数据导出功能
/*
unsigned int __stdcall ExportJointThread(PVOID pParam)
{
    int id = (int)pParam;
    std::string filepath = "..\\..\\resource\\ExportData\\";
    std::string filename = std::to_string(id) + "joint_data";
    std::string prefix = ".txt";
    std::string fullpath = filepath + filename + prefix;
    ofstream joint_value(fullpath, ios::app | ios::out);
    joint_value << "shoulder "
                << " elbow " << endl;
    double joint[2];
    while (true) {
        if (is_exit_thread_) break;
        ControlCard::GetInstance().GetEncoderData(joint);
        joint_value << joint[0] << "   " << joint[1] << endl;
    }
    joint_value.close();
}
void ActiveControl::StartExportJointdata(int id)
{
    is_exit_thread_ = false;
    (HANDLE) _beginthreadex(NULL, 0, ExportJointThread, void *id, 0, NULL);
}
*/

// void ActiveControl::TorqueExport()
// {
//     ofstream dataFile2;
//     dataFile2.open("torque.txt", ofstream::app);
//     dataFile2 << "torque_sholuder"
//               << "   "
//               << "torque_elbow" << endl;
//     for (int i = 0; i < torque_data[0].size(); ++i) {
//         dataFile2 << torque_data[0][i] << "        " << torque_data[1][i] << endl;
//     }
//     dataFile2.close();
// }