/********************************************************************
Created:	2012.5.23  10:46
Author:		hps

Purpose:	��ʾ Setting.mdb �е� LayerCodeDefine ��
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
		/// ��ʾ Setting.mdb �е� LayerCodeDefine ��
		/// </summary>
		class DCICOMMONEXT_MODULE_EXPIMP CMdbLayerCodeDefine : public CAdoTable
		{
			// �ֶ����Ƴ���
		public:
			static const CString MdbLayerCodeDefineTableName;
			static const CString ID;

		public:
			CMdbLayerCodeDefine(void);
			virtual ~CMdbLayerCodeDefine(void);

			static CMdbLayerCodeDefine& Instance();

			/// <summary>
			/// ����ָ������Ŀ��ţ���ȡһ���¼�С�
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а��� 4 ���ֶΣ�[layerName��propName��propVal��code]</remarks>
			/// <param name="output">[O] ������������ļ�¼�С�</param>
			/// <param name="projectId">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.26   17:01"/>
			bool GetByProject(List<CAdoRow*> &output, long projectId);
			static bool SGetByProject(List<CAdoRow*> &output, long projectId);

			/// <summary>
			/// ����ָ������Ŀ��ź�CADͼ��������ȡһ���¼�С�
			/// </summary>
			/// <remarks>�����ÿһ�м�¼�а��� 3 ���ֶΣ�[propName��propVal��code]</remarks>
			/// <param name="output">[O] ������������ļ�¼�С�</param>
			/// <param name="projectId">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <param name="layerName">[I] ָ��Ҫ��ѯ��CADͼ������ơ�</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.24   12:03"/>
			bool GetByProjectAndLayer(List<CAdoRow*> &output, long projectId, const CString &layerName);
			static bool SGetByProjectAndLayer(List<CAdoRow*> &output, long projectId, const CString &layerName);

			/// <summary>
			/// ����ָ������Ŀ��ź�����ֵ����ȡ��һ��ƥ���¼�е� Code �е�ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="outCode">[O] ��� code �е�ֵ��</param>
			/// <param name="projectId">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <param name="propVal">[I] ָ��Ҫ��ѯ��ѯ������ֵ��</param>
			/// <returns>��ִ�гɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.26   16:44"/>
			bool GetByProjectAndPropVal(CString &outCode, long projectId, const CString &propVal);
			static bool SGetByProjectAndPropVal(CString &outCode, long projectId, const CString &propVal);

			bool GetByCode(CAdoRow *&pOutRow, const CString &code, const CString &layernmae);
			static bool SGetByCode(CAdoRow *&pOutRow, const CString &code, const CString &layernmae);

		private:
			virtual int OnReflectTo(); 
		};

	}
}