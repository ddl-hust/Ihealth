#ifndef ACTIVECONTROL_H
#define ACTIVECONTROL_H
#include "control_card.h"
#include "boundarydetection.h"
#include <vector>
#include "matrix.h"

class ActiveControl
{
public:
    ActiveControl();
    ~ActiveControl();
    void LoadParamFromFile();
    void StartMove();
    void StopMove();
    void Step(); // get six_dim_force data
    void PressureStep(); // get pressure data
    bool IsFire(); // get gripper sensor to decide wheather to start active game, now Depreciated
    void CalculatePlaneXY(short Axis_X, short Axis_Y, double XY[2]); //get end-efftor position
    void CalculateRagXY(double XY[2]); //获取游戏中抹布位置
    void SetDamping(float FC = 0.1);
    // set the max value and control sensitvity for active joint
    void SetSAAMax(double saa);
    void SetSFEMax(double sfe);
    void SetArmSensitivity(double arm_senitivity);
    void SetShoulderSensitivity(double shoulder_senitivity);
    void ActiveTorqueToAllTorque(double torque[2], double alltorque[5]);
    // void TorqueStep();
    // export torque sensor to txt file
    // void TorqueExport();
    // boundaryDetection detect; // ??? why need this  my this is left for torque solution


public:
    bool is_exit_thread_;
    bool is_moving_;
    double six_dimension_offset_[6];
    double elbow_offset_[2];
    double torque_offset_[2];
    double cycle_time_in_second_;

private:
    void MoveInNewThread();
    // unsigned int __stdcall ActiveMoveThread(PVOID pParam);
    void ExitMoveThread();
    void ActMove(); // send instruction to two active motor call APS's API
    void Trans2FilterForPressure(double TransData[2], double FiltedData[2]); // filter pressure
    // void Raw2Trans(double RAWData[6], double DistData[6]);  //change the raw six_dim_force through tranformation matrix to handle frame
    // void Trans2Filter(double TransData[6], double FiltedData[6]); //filter the transformed six_dim_force
    // void FiltedVolt2Vel(double FiltedData[6]); //not figure what's the function ???
    void MomentCalculation(double ForceVector, double &vel); // ???
    //将传感器的数据处理成两个二维矢量，由于矢量只在两个方向上有作用，故需输出4个数据。这里要先知道传感器的安装位置
    // void SensorDataToForceVector(double shouldersensordata[4], double elbowsensordata[4], double ForceVector[4]);


private:
    HANDLE move_thread_handle_;
    Matrix3d rotate_matrix_;
    //手柄坐标系下手柄坐标系原点到六维力坐标系原点的向量
    Vector3d force_position_;

    double shoulder_angle_max_;
    double elbow_angle_max_;
    double elbow_Sensitivity_ = 0;
    double shoulder_Sensitivity_ = 0;
    static double six_dimforce[6];
    double joint_angle[2];
};

#endif // ACTIVECONTROL_H
