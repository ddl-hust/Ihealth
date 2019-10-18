#pragma once
#include "RFMYSQL.h"


#define BLOOD_A 0
#define BLOOD_B 1
#define BLOOD_AB 2
#define BLOOD_O 3

#define BLOOD_A_STRING _T("A")
#define BLOOD_B_STRING _T("B")
#define BLOOD_AB_STRING _T("AB")
#define BLOOD_O_STRING _T("O")

struct ModifyPWDInfo
{
	int loginid;

	std::wstring pwd;
	std::wstring oldpwd;
};

struct LoginInfo
{
	std::wstring usrname;
	std::wstring usrpwd;
	int			role;
	int			logined;

	std::wstring login_user;

	int doctorid;
	int login_id;
	int hospitalid;
	std::wstring employeenumber;
	std::wstring name;
	std::wstring sex;
	std::wstring birthday;
	std::wstring certid;
	std::wstring telephone;
	int			 bloodtype;
	std::wstring address;
	std::wstring education;
	std::wstring nation;
	std::wstring birthplace;
	std::wstring householdprop;
	std::wstring department;
	std::wstring group;
	std::wstring positon;
	std::wstring entrydate;
	int flag;
	std::wstring createtime;
};

struct LoadPatientParam
{
	int hospitalid;
	int doctorid;
	
	int start;
	int num;
};

struct SearchPatientParam
{
	std::wstring patientname;
};

struct NextPatientResult
{
	std::wstring patientid;
};


struct PatientFilterParam
{
	int hospitalid;
	int doctorid; 
	std::wstring name;
	std::wstring sex;
	std::wstring age_from;
	std::wstring age_to;
	std::wstring create_from;
	std::wstring create_to;
};

struct ExportPatientParam
{
	std::wstring path;
	std::wstring savepath;
	int patientid;
	int hospitalid;
	int doctorid;
};


struct PatientInfo
{
	int id;
	int hospitalid;
	int doctorid;
	std::wstring name;
	std::wstring sex;
	int age;
	std::wstring createtime;
	std::wstring lasttreattime;
	std::wstring totaltreattime;
	std::wstring recoverdetail;
	std::wstring remarks;
	int flag;
	double SAA_ROM; // �粿����չ�����˶���Χ�����ֵ����Ӧ�粿�Ƕȣ�?
	double SFE_ROM; // �粿�������˶���Χ���ֵ����Ӧ�ⲿ�Ƕȣ�?
	double Arm_Sensitivity; // the parm to adjust arm force->velocity
	double Shoulder_Sensitivity; 
};

struct LoadPatientResult
{
	int num;
	std::list<PatientInfo> patients;

	int updateui;
};

struct ExportPatientResult
{
	int num;
	std::list<PatientInfo> patients;

	int updateui;
	std::wstring path;
	std::wstring savepath;
};

struct PatientTrainInfo
{
	int id;
	std::wstring name;
	std::wstring sex;
	std::wstring age;
	std::wstring createtime;
};

struct LoadPatientTrainInfoParam
{
	int doctorid;
	int hospitalid;
};

struct LoadPatientTrainInfoResult
{
	std::list<PatientTrainInfo> patienttraininfos;
};

#define RF_ACTIVE_TRAIN_MODE 0
#define RF_PASSIVE_TRAIN_MODE 1
#define RF_EMG_TRAIN_MODE 2
#define RF_EYE_TRAIN_MODE 3

#define RF_TRAINTYPE_STRING_ZD _T("����ѵ��")
#define RF_TRAINTYPE_STRING_BD _T("����ѵ��")
#define RF_TRAINTYPE_STRING_YD _T("�۶�ѵ��")
#define RF_TRAINTYPE_STRING_EMG _T("EMGѵ��")

struct PatientTrainDetails
{
	int id;
	int patientid;
	std::wstring traindate;
	std::wstring content;
	std::wstring duration;
	//int score;
	std::wstring traintime;
	std::wstring createtime;
	int flag;

	std::wstring traintype;	// 0����ѵ�� 1 ����ѵ�� 2 EMGģʽ 3 �۶�ģʽ
	std::wstring game;
	std::wstring nandu;
	std::wstring defen;


	std::wstring totaltreattime;
	std::wstring lasttreattime;
	std::wstring recoverdetail;

	std::vector<double> target_pos[2];	// ����ѵ�� �˶��Ƕ� 0 ��ؽ�?1 λ��ؽ�?
	std::vector<double> target_vel[2];  //			�˶��ٶ� ͬ��

	std::vector<double> emg_signal[4];	// EMGѵ��
	std::vector<double> emg_angle[2];

	std::vector<double> target_angle;	// ����ѵ�� �˶��Ƕ�
	std::vector<double> target_wl;		//			����
};

struct LoadPatientTrainDetailsParam
{
	int hospitalid;
	int doctorid;
	int patientid;
};

struct LoadPatientTrainDetailsResult
{
	std::list<PatientTrainDetails> patienttraindetails;
};

// ��Ӧ�����ݿ��е�passivetrain�����ڼ�¼
// ����ģʽ��һ��¼�Ƶ��˶�����
struct PassiveTrainInfo
{
	std::wstring id;
	std::wstring name;
	std::wstring traintype;
	std::wstring timelen;
	//std::wstring movements;

	std::vector<double> target_pos[2];	// �˶��Ƕ� 0 ��ؽ�?1 λ��ؽ�?
	std::vector<double> target_vel[2];  // �˶��ٶ� ͬ��
};

struct LoadPassiveTrainInfoResult
{
	std::list<PassiveTrainInfo> passivetraininfos;
};

struct LoadPatientTrainDataParam
{
	int traindetailid;
};

struct PatientTrainData
{
	int id;
	int traindetailid;
	int timetrace;
	int tracevalue;
	int traintype;		// 1 �ؽ��˶��Ƕ� 2 ���� 3 EMG�ź� 4 ��ؽ�����?5 ��ؽ�����?
};

struct LoadPatientTrainDataResult
{
	std::list<PatientTrainData> patienttraindatas;
};

struct TTSSampleData
{
	std::string text;
	std::string filepath;
};

struct EvaluationRecordData
{
	int id;
	int evalid;
	std::wstring item;
	std::wstring result;
};

struct EvaluationData
{
	int id;
	int doctorid;
	int patientid;
	std::wstring date;
	std::wstring name;
	std::wstring score;
	int	type;	// 1 fma 2 mas 3 �˶�

	std::list<EvaluationRecordData> datas;
};

struct LoadEvaluationParam
{
	int doctorid;
	int type;
};

struct LoadEvaluationResult
{
	int nextid;
	std::list<EvaluationData> datas;
};

struct EvaluationYDGN
{
	double zb1;
	double zb2;
	double zb;
	double jb1;
	double jb2;
	double jb;
	double cs;
	double wl;

	double score;
};

class RFMySQLThread : public CThreadBase
{
public:
	RFMYSQL m_db;

public:
	RFMySQLThread(void);
	~RFMySQLThread(void);


public:
	// �������ݿ��߳�ʵ��
	static RFMySQLThread* Create();
	// �������ݿ��߳���Դ
	static void Release(RFMySQLThread *&pThread);
	// �������ݿ�����
	static int Connect(EventArg *pArg);
	// ���ݿ�����
	static int ReConnect(EventArg *pArg);
	// ���ݿ��û���¼
	static int Login(EventArg *pArg);

	static int Load(EventArg* pArg);
	static int Save(EventArg* pArg);
	static int Add(EventArg* pArg);
	static int AddMultiPatients(EventArg* pArg);
	static int Delete(EventArg* pArg);
	static int NextPatientID(EventArg* pArg);

	static int ModifyPWD(EventArg* pArg);
	static int SaveDoctor(EventArg* pArg);

	static int SearchPatient(EventArg* pArg);
	static int FilterPatient(EventArg* pArg);
	static int ExportPatient(EventArg* pArg);

	static int AddPatientTrainDetails(EventArg* pArg);
	static int LoadPatientTrainInfo(EventArg* pArg);
	static int LoadPatientTrainDetails(EventArg* pArg);
	static int LoadPassiveTrainInfo(EventArg* pArg);
	// ������ģʽ��¼�Ƶ��������ӵ����ݿ���
	static int AddPassiveTrainInfo(EventArg* pArg);
	static int DeletePassiveTrainInfo(EventArg *pArg);
	static int LoadPatientTrainData(EventArg* pArg);

	static void LoadEvaluationRecords(int id, std::list<EvaluationRecordData>& datas);
	static int LoadEvaluationDatas(EventArg* pArg);
	static int AddEvaluationData(EventArg* pArg);

	static int TTSSample(EventArg* pArg);

	PatientFilterParam m_patientFilter;
};
