/********************************************************************
Created:	2012.5.23  10:46
Author:		hps

Purpose:	��ʾ Setting.mdb �е� contrastProp ��
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
		/// ��ʾ Setting.mdb �е� contrastProp ��
		/// </summary>
		class DCICOMMONEXT_MODULE_EXPIMP CMdbContrastProp : public CAdoTable
		{
			// �ֶ����Ƴ���
		public:
			static const CString MdbContrastPropTableName;

		public:
			CMdbContrastProp(void);
			virtual ~CMdbContrastProp(void);

			static CMdbContrastProp& Instance();

			/// <summary>
			/// ���� contrastLayer ����ָ���� sdeLayer �е�ֵ�� contrastProp ���� specialVal �е�ֵ����ȡ contrastProp ���� sdeProp �ж�Ӧ��ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="output">[O] ������Զ��ձ� contrastProp �� sdeProp �ж�Ӧ��ֵ��</param>
			/// <param name="strSdeLayer">[I] ָ��Ҫ��ѯ�� sde ͼ������</param>
			/// <param name="strSpecialValue">[I] ָ�� contrastProp ���� specialVal �е�ֵ��</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.19   16:20"/>
			bool GetSdePropertyBySpecialValue(CString &output, const CString &strSdeLayer, const CString &strSpecialValue);
			static bool SGetSdePropertyBySpecialValue(CString &output, const CString &strSdeLayer, const CString &strSpecialValue);

			/// <summary>
			/// ��ȡָ���� sde ͼ��������ֶ���Ϣ��
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а��� 3 ���ֶΣ�[sdeProp��dataType��length]</remarks>
			/// <param name="output">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ�� sde ͼ�����ڵ���Ŀ��š�</param>
			/// <param name="strSdeLayer">[I] ָ��Ҫ��ѯ�� sde ͼ�����ơ�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.19   16:25"/>
			bool GetAllSdeProperty(List<CAdoRow*> &output, long projectID, const CString &strSdeLayer);
			static bool SGetAllSdeProperty(List<CAdoRow*> &output, long projectID, const CString &strSdeLayer);

			
			/// <summary>
			/// ��ȡָ���� sde ͼ���е����������ֶ���Ϣ��
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а��� 3 ���ֶΣ�[sdeProp��dataType��length]</remarks>
			/// <param name="output">[O] �����¼�С�</param>
			/// <param name="projectID">[I] ָ�� sde ͼ�����ڵ���Ŀ��š�</param>
			/// <param name="strSdeLayer">[I] ָ��Ҫ��ѯ�� sde ͼ�����ơ�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.8.15   10:11"/>
			bool GetAllSpecialSdeProperty(List<CAdoRow*> &output, long projectID, const CString &strSdeLayer);
			static bool SGetAllSpecialSdeProperty(List<CAdoRow*> &output, long projectID, const CString &strSdeLayer);

			/// <summary>
			/// ��ȡָ���� sde ͼ�����Զ�����Ϣ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="output">�����ÿһ�м�¼�а��� 3 ���ֶΣ�[contrastProp.sdeProp��contrastProp.cadProp��contrastProp.specialVal]</param>
			/// <param name="projectID"></param>
			/// <param name="strSdeLayer"></param>
			/// <returns></returns>
			/// <author name="hps" date="2012.7.23   10:43"/>
			bool GetContrastCadInfoForSdeLayer(List<CAdoRow*> &output, long projectID, const CString &strSdeLayer);
			static bool SGetContrastCadInfoForSdeLayer(List<CAdoRow*> &output, long projectID, const CString &strSdeLayer);
			
		private:
			virtual int OnReflectTo(); 
		};

	}
}