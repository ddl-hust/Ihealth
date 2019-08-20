/********************************************************************
Created:	2012.5.23  10:46
Author:		hps

Purpose:	��ʾ Setting.mdb �е� objectType ����ͼ���
Remark:		
*********************************************************************/

#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include "List.h"
using namespace Ext::Collection;

#include "AdoRow.h"
#include "AdoTable.h"
using namespace Ext::Ado;

namespace Dci
{
	namespace SettingMDB
	{

		/// <summary>
		/// ��ʾ Setting.mdb �е� projectType ��
		/// </summary>
		class DCICOMMONEXT_MODULE_EXPIMP CMdbObjectType : public CAdoTable
		{
			// �ֶ����Ƴ���
		public:
			static const CString MdbObjectTypeTableName;
			static const CString ID;
			static const CString LayerName;

		public:
			CMdbObjectType(void);
			virtual ~CMdbObjectType(void);

			static CMdbObjectType& Instance();

			/// <summary>
			/// ��ȡָ����Ŀ������ͼ����Ϣ��
			/// </summary>
			/// <remarks>����� CAdoRow ����ļ�¼�У�[ID, prjId, layerName, entityType, color, linetype]</remarks>
			/// <param name="rows">[O] �����Ӧ��Ŀ������ͼ�㡣ÿһ�б�ʾ objectType ���е�һ����¼��</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.7.13   10:45"/>
			void GetProjectLayers(List<CAdoRow*> &rows, const int& projectID);
			static void SGetProjectLayers(List<CAdoRow*> &rows, const int& projectID);

			/// <summary>
			/// ��ȡָ����Ŀ������ͼ������
			/// </summary>
			/// <remarks></remarks>
			/// <param name="layerNames">[O] �����Ӧ��Ŀ������ͼ������</param>
			/// <param name="projectID">[I] ָ��Ҫ��ѯ����Ŀ��š�</param>
			/// <returns></returns>
			/// <author name="hps" date="2012.7.13   10:46"/>
			void GetProjectLayers(List<CString> &layerNames, const int& projectID);
			static void SGetProjectLayers(List<CString> &layerNames, const int& projectID);

			CAdoRow* GetOneRow(const int& projectID, const CString &layerName);
			static CAdoRow* SGetOneRow(const int& projectID, const CString &layerName);

		private:
			virtual int OnReflectTo(); 
		};
	}
}