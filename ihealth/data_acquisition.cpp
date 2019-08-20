#include "data_acquisition.h"

#include <iostream>
#include<Windows.h>
#include <Eigen/core>

using namespace Eigen;
using namespace std;

const char *DataAcquisition::kTorqueChannel = "dev2/ai4:5";
const char *DataAcquisition::kPullSensorChannel = "dev2/ai0:3";
const char *DataAcquisition::kGripChannel = "dev2/ai6";
const char *DataAcquisition::kSixDimensionForceChannel = "dev1/ai0:5";
const char *DataAcquisition::kPressureForceChannel= "Dev3/ai1:2";
const double DataAcquisition::kRawToReal = 2.0;

Eigen::Matrix<double, 6, 6> DataAcquisition::kTransformMatrix = MatrixXd::Zero(6, 6);

DataAcquisition::DataAcquisition() {
	int status;
	//六维力
	status = DAQmxCreateTask("", &s_task_handle);
	status = DAQmxCreateAIVoltageChan(s_task_handle, kSixDimensionForceChannel, "",
		DAQmx_Val_Diff, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(s_task_handle, NULL, 100, DAQmx_Val_Rising,
		DAQmx_Val_ContSamps, 2);

	status = DAQmxSetReadRelativeTo(s_task_handle, DAQmx_Val_MostRecentSamp);
	status = DAQmxSetReadOffset(s_task_handle, 0);
	status = DAQmxStartTask(s_task_handle);
	status = DAQmxStopTask(s_task_handle);

	//压力传感器
	status = DAQmxCreateTask("", &p_task_handle);
	status = DAQmxCreateAIVoltageChan(p_task_handle, kPressureForceChannel, "",
		DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(p_task_handle, NULL, 100, DAQmx_Val_Rising,
		DAQmx_Val_ContSamps, 2);

	status = DAQmxSetReadRelativeTo(p_task_handle, DAQmx_Val_MostRecentSamp);
	status = DAQmxSetReadOffset(p_task_handle, 0);
	status = DAQmxStartTask(p_task_handle);
	status = DAQmxStopTask(p_task_handle);

	//力矩传感器
	//status = DAQmxCreateTask("TorqueDataTask", &t_task_handle);
	//status = DAQmxCreateAIVoltageChan(t_task_handle, kTorqueChannel, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	//status = DAQmxCfgSampClkTiming(t_task_handle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);

	//status = DAQmxSetReadRelativeTo(t_task_handle, DAQmx_Val_MostRecentSamp);
	//status = DAQmxSetReadOffset(t_task_handle, 0);
	//status = DAQmxStartTask(t_task_handle);
	//status = DAQmxStopTask(t_task_handle);


	//初始化变换矩阵
	kTransformMatrix << -0.02387, -0.16164, 0.65185, 48.29934, 0.22454, -48.21503,
		-0.79366, -55.63349, -0.38984, 27.64338, 0.06443, 27.76042,
		65.56995, -0.53484, 65.97331, -3.97922, 65.95160, 1.01335,
		-0.02190, 0.02187, -1.16058, 0.03057, 1.13361, -0.00412,
		1.31922, -0.01212, -0.69040, 0.02994, -0.65557, 0.00866,
		0.01390, 1.00881, 0.00664, 1.00606, -0.00454, 1.02667;
}



DataAcquisition &DataAcquisition::GetInstance() {
	static DataAcquisition instance;
	return instance;
}

void DataAcquisition::AcquisiteTorqueData() {
	//TaskHandle taskHandle = 0;
	int32 read = 0;
	int status = 0;
	//double torque_data[2]{ 0 };
	//status = DAQmxCreateTask("TorqueDataTask", &taskHandle);
	//status = DAQmxCreateAIVoltageChan(taskHandle, kTorqueChannel, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	//status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	//status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(t_task_handle, 10, 0.2, DAQmx_Val_GroupByScanNumber, torque_data, 20, &read, NULL);
	//status = DAQmxStopTask(taskHandle);
	//status = DAQmxClearTask(taskHandle);

	/*这里需要根据实际的连线确定正确的顺序*/
	//shoulder_raw_torque_ = torque_data[1];
	//elbow_raw_torque_ = torque_data[0];
}

void DataAcquisition::AcquisiteTorqueData(double torquedata[2]) {
	int32 read = 0;
	int status = 0;
	double torque_data[2]{ 0 };

	//status = DAQmxCreateTask("TorqueDataTask", &taskHandle);
	//status = DAQmxCreateAIVoltageChan(taskHandle, kTorqueChannel, "TorqueDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	//status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	//status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(t_task_handle, 1, 0.2, DAQmx_Val_GroupByScanNumber, torque_data, 2, &read, NULL);
	//status = DAQmxStopTask(taskHandle);
	//status = DAQmxClearTask(taskHandle);

	torquedata[0] = torque_data[0];
	torquedata[1] = torque_data[1];
}

void DataAcquisition::AcquisitePullSensorData() {
	TaskHandle taskHandle = 0;
	int32 read = 0;
	int status = 0;
	double pull_sensor_data[4]{ 0 };
	status = DAQmxCreateTask("PullDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, kPullSensorChannel, "PullDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, pull_sensor_data, 4, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);

	/*这里需要根据实际连线确定正确的顺序*/
	shoulder_raw_forward_pull_ = pull_sensor_data[0];
	shoulder_raw_backward_pull_ = pull_sensor_data[1];
	elbow_raw_forward_pull_ = pull_sensor_data[2];
	elbow_raw_backward_pull_ = pull_sensor_data[3];
}

void DataAcquisition::AcquisiteTensionData(double tension_output[2]) {
	//TaskHandle taskHandle = 0;
	int32 read = 0;
	int status = 0;
	double tension_data[2]{ 0 };
	//status = DAQmxCreateTask("PressureDataTask", &taskHandle);
	//status = DAQmxCreateAIVoltageChan(taskHandle, kPressureForceChannel, "PressureDataChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	//status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising,DAQmx_Val_ContSamps, 2);
	//status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(p_task_handle, 1, 0.2, DAQmx_Val_GroupByScanNumber, tension_data, 2, &read, NULL);
	//status = DAQmxStopTask(taskHandle);
	//status = DAQmxClearTask(taskHandle);

	for (int i = 0; i < 2; ++i) {
		tension_output[i] = tension_data[i];
	}
}

void DataAcquisition::AcquisiteSixDemensionData(double output_buf[6]) {
	TaskHandle taskHandle = 0;
	int32 read = 0;
	int status = 0;
	double raw_data[6];
	/*status = DAQmxCreateTask("SixDemensionDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, kSixDimensionForceChannel, "SixDimensionChannel", DAQmx_Val_Diff, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 100, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 1);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, raw_data, 6, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);*/
	status = DAQmxReadAnalogF64(s_task_handle, 1, 0.2, DAQmx_Val_GroupByScanNumber, raw_data, 6, &read, NULL);
	//cout << "status is : " << status << endl;
	//char buffer[256];
	//DAQmxGetErrorString(status, buffer, 256);
	//cout << "error code is : " << buffer << endl;

	//AllocConsole();
	//freopen("CONOUT$", "w", stdout);
	//printf("%lf    %lf    %lf    %lf    %lf    %lf \n", raw_data[0], raw_data[1], raw_data[2], raw_data[3], raw_data[4], raw_data[5]);

	Matrix<double, 6, 1> dat;
	for (int i = 0; i < 6; ++i) {
		dat(i, 0) = raw_data[i];
	}

	VectorXd result(6);
	result = kTransformMatrix * dat;


	for (int i = 0; i < 6; ++i) {
		output_buf[i] = result(i);
	}
}

void DataAcquisition::AcquisiteGripData(double grip[1]) {
	TaskHandle taskHandle = 0;
	int32 read = 0;
	int status = 0;
	status = DAQmxCreateTask("GripDataTask", &taskHandle);
	status = DAQmxCreateAIVoltageChan(taskHandle, kGripChannel, "GripChannel", DAQmx_Val_RSE, -10, 10, DAQmx_Val_Volts, NULL);
	status = DAQmxCfgSampClkTiming(taskHandle, "OnboardClock", 1000, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 10);
	status = DAQmxStartTask(taskHandle);
	status = DAQmxReadAnalogF64(taskHandle, 1, 0.2, DAQmx_Val_GroupByScanNumber, grip, 1, &read, NULL);
	status = DAQmxStopTask(taskHandle);
	status = DAQmxClearTask(taskHandle);
}

double DataAcquisition::ShoulderTorque() {
	return kRawToReal * shoulder_raw_torque_;
}

double DataAcquisition::ElbowTorque() {
	return kRawToReal * elbow_raw_torque_;
}

double DataAcquisition::ShoulderForwardPull() {
	return kRawToReal * shoulder_raw_forward_pull_;
}

double DataAcquisition::ShoulderBackwardPull() {
	return kRawToReal * shoulder_raw_backward_pull_;
}

double DataAcquisition::ElbowForwardPull() {
	return kRawToReal * elbow_raw_forward_pull_;
}

double DataAcquisition::ElbowBackwardPull() {
	return kRawToReal * elbow_raw_backward_pull_;
}

bool DataAcquisition::StartTask() {
	int status;
	status = DAQmxStartTask(p_task_handle);
	cout << status << endl;
	return status == 0;
}

bool DataAcquisition::StopTask() {
	int status;
	status = DAQmxStopTask(p_task_handle);
	return status == 0;
}

bool DataAcquisition::StartTorqueTask() {
	int status;
	status = DAQmxStartTask(t_task_handle);
	cout << status << endl;
	return status == 0;
}

bool DataAcquisition::StopTorqueTask() {
	int status;
	status = DAQmxStopTask(t_task_handle);
	return status == 0;
}

bool DataAcquisition::StartSixDemTask() {
	int status;
	status = DAQmxStartTask(s_task_handle);
	cout << status << endl;
	return status == 0;
}

bool DataAcquisition::StopSixDemTask() {
	int status;
	status = DAQmxStopTask(s_task_handle);
	return status == 0;
}