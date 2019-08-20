#pragma once
#include <NIDAQmx.h>
#include <Eigen/core>

class DataAcquisition {
public:
	static DataAcquisition &GetInstance();
	DataAcquisition(const DataAcquisition &) = delete;
	DataAcquisition(DataAcquisition &&) = delete;
	DataAcquisition &operator=(const DataAcquisition &) = delete;
	DataAcquisition &operator=(DataAcquisition &&) = delete;

	void AcquisiteTorqueData();
	void AcquisiteTorqueData(double torquedata[2]);
	void AcquisitePullSensorData();
	void AcquisiteSixDemensionData(double output_buf[6]);
	//这里尝试下把肩肘的数据采集放在一起，感觉这样性能可以提升
	void AcquisiteTensionData(double tension_output[2]);
	void AcquisiteGripData(double grip[1]);
	double ShoulderTorque();
	double ElbowTorque();
	double ShoulderForwardPull();
	double ShoulderBackwardPull();
	double ElbowForwardPull();
	double ElbowBackwardPull();

	bool StartTask();
	bool StopTask();
	bool StartTorqueTask();
	bool StopTorqueTask();
	bool StartSixDemTask();
	bool StopSixDemTask();

public:
	double torque_data[20];

private:
	DataAcquisition();

private:
	TaskHandle s_task_handle;
	TaskHandle p_task_handle;
	TaskHandle t_task_handle;

	double shoulder_raw_torque_ = 0.0;
	double elbow_raw_torque_ = 0.0;

	double shoulder_raw_forward_pull_ = 0.0;
	double shoulder_raw_backward_pull_ = 0.0;
	double elbow_raw_forward_pull_ = 0.0;
	double elbow_raw_backward_pull_ = 0.0;

	static const char *kTorqueChannel;
	static const char *kPullSensorChannel;
	static const char *kSixDimensionForceChannel;
	static const char *kGripChannel;
	static const char *kPressureForceChannel;
	static const double kRawToReal;

	static Eigen::Matrix<double, 6, 6>  kTransformMatrix;

};
