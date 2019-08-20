/********************************************************************
Created:	2012.5.23  10:46
Author:		hps

Purpose:	��ʾ Setting.mdb �е� contrastLayer ��
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
		/// ��ʾ Setting.mdb �е� contrastLayer ��
		/// </summary>
		class DCICOMMONEXT_MODULE_EXPIMP CMdbContrastLayer : public CAdoTable
		{
			// �ֶ����Ƴ���
		public:
			static const CString MdbContrastLayerTableName;

		public:
			CMdbContrastLayer(void);
			virtual ~CMdbContrastLayer(void);

			static CMdbContrastLayer& Instance();


			/// <summary>
			/// ��ȡָ����Ŀ�е����ʱ���ж��յ� SDE ͼ����Ϣ��
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а��������ֶΣ�[sdeLayer��entityType]</remarks>
			/// <param name="output">[O] �����¼�С�</param>
			/// <param name="projectId">[I] ָ����Ŀ��ţ��� projectType �е� ID �ֶε�ֵ��</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.19   15:35"/>
			bool GetAllSdeLayers(List<CAdoRow*> &output, long projectId);
			static bool SGetAllSdeLayers(List<CAdoRow*> &output, long projectId);

			/// <summary>
			/// ������Ŀ��ź�ָ����sdeͼ��������ȡcad�Ķ�����Ϣ��
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а��� 3 ���ֶΣ�[layerName��propName��propVal]</remarks>
			/// <param name="output">[O] �����¼�С�</param>
			/// <param name="projectId">[I] ָ����Ŀ��ţ��� projectType �е� ID �ֶε�ֵ��</param>
			/// <param name="strSdeLayer">[I] ָ��Ҫ��ѯ��SDEͼ������ơ�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.20   10:37"/>
			bool GetContrastCadInfoForSdeLayer(List<CAdoRow*> &output, long projectId, const CString &strSdeLayer);
			static bool SGetContrastCadInfoForSdeLayer(List<CAdoRow*> &output, long projectId, const CString &strSdeLayer);

			
			/// <summary>
			/// ���� SDE ͼ�����ƻ�ȡ��Ŀ���ͱ��
			/// </summary>
			/// <remarks>���ҽ��� contrastLayer ���� sdeLayer �ֶ�ֵΨһ��ʱ����ã�����</remarks>
			/// <param name="outputProjectId">[O] �����Ŀ��ţ��� projectType �е� ID �ֶε�ֵ��</param>
			/// <param name="strSdeLayer">[I] ָ��Ҫ��ѯ��SDEͼ������ơ�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2013.2.20   13:47"/>
			bool GetProjectIdBySdeLayer(long &outputProjectId, const CString &strSdeLayer);
			static bool SGetGetProjectIdBySdeLayer(long &outputProjectId, const CString &strSdeLayer);
			
		private:
			virtual int OnReflectTo(); 
		};

	}
}