/********************************************************************
Created:	2012.5.23  10:46
Author:		hps

Purpose:	��ʾ Setting.mdb �е� CheckedOption ��
Remark:		
*********************************************************************/

#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include "AdoTable.h"
using namespace Ext::Ado;

namespace Dci
{
	namespace SettingMDB
	{

		/// <summary>
		/// ��ʾ Setting.mdb �е� CheckedOption ��
		/// </summary>
		class DCICOMMONEXT_MODULE_EXPIMP CMdbCheckedOption : public CAdoTable
		{
			// �ֶ����Ƴ���
		public:
			static const CString MdbCheckedOptionTableName;

		public:
			CMdbCheckedOption(void);
			virtual ~CMdbCheckedOption(void);

			static CMdbCheckedOption& Instance();

			/// <summary>
			/// ��ȡ����ֵ��⡱����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:02"/>
			bool GetCheckNullOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckNullOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ���ظ��ԡ�����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:05"/>
			bool GetCheckRepeatOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckRepeatOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ���պ��ԡ�����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:06"/>
			bool GetCheckClosedOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckClosedOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ�����֤֤�š��ռ���š����֤�˷�ʱ������ԡ�����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:07"/>
			bool GetCheckZhuhaiRelationOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckZhuhaiRelationOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ������ֵ�淶�Լ�⡱����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:07"/>
			bool GetCheckRegexpOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckRegexpOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ��������⡱����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:08"/>
			bool GetCheckAttachmentOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckAttachmentOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ���̶����ȡ�����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:09"/>
			bool GetCheckFixLengthOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckFixLengthOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ���õش������õ����ʹ����ԡ�����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.30   15:10"/>
			bool GetCheckYDDaimaAndXingzhiOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckYDDaimaAndXingzhiOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ��ɾ����ȫ�ص�����ߡ�����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.10.8   14:34"/>
			bool GetRemoveEmbedLineOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetRemoveEmbedLineOptions(List<CAdoRow*> &outOptions, long projectID);

			/// <summary>
			/// ��ȡ�����߹ܵ�����Լ�⡱����
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а���ȫ���ֶΣ�[ID��ProjectID��LayerName��PropertyName��CheckType��Expected��ErrorMessage��Summary]</remarks>
			/// <param name="outOptions">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.10.12   9:12"/>
			bool GetCheckPipeLineMatchOptions(List<CAdoRow*> &outOptions, long projectID);
			static bool SGetCheckPipeLineMatchOptions(List<CAdoRow*> &outOptions, long projectID);
			
		private:
			virtual int OnReflectTo(); 
		};

	}
}