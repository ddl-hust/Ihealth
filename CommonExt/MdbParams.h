/********************************************************************
Created:	2012.5.23  10:46
Author:		hps

Purpose:	��ʾ Setting.mdb �е� params ��
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
		/// ��ʾ Setting.mdb �е� params ��
		/// </summary>
		class DCICOMMONEXT_MODULE_EXPIMP CMdbParams : public CAdoTable
		{
			// �ֶ����Ƴ���
		public:
			static const CString MdbParamsTableName;
			static const CString ID;

		public:
			CMdbParams(void);
			virtual ~CMdbParams(void);

			static CMdbParams& Instance();

			/// <summary>
			/// ��ȡָ�������Ĳ���ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strParamName">[I] ָ��Ҫ�����Ĳ�������</param>
			/// <returns>���ض�Ӧ�Ĳ���ֵ����ָ���Ĳ����������ڣ��򷵻ؿ��ַ�����</returns>
			/// <author name="hps" date="2012.7.13   10:39"/>
			CString GetValue(const CString &strParamName);
			static CString SGetValue(const CString &strParamName);

			/// <summary>
			/// ��ȡָ�������Ĳ���ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="output">[O] �����Ӧ�Ĳ���ֵ����ָ���Ĳ����������ڣ�output ���ֲ��䡣</param>
			/// <param name="strParamName">[I] ָ��Ҫ�����Ĳ�������</param>
			/// <returns>�����������ڲ��ɹ���ȡ����ֵ���򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.13   10:40"/>
			bool GetValue(CString &output, const CString &strParamName);
			static bool SGetValue(CString &output, const CString &strParamName);

			/// <summary>
			/// ����ָ���������Ĳ���ֵ��
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strParamName">[I] ָ��Ҫ�޸ĵĲ�������</param>
			/// <param name="strParamValue">[I] ָ����������ֵ��</param>
			/// <returns>�����������ڲ��޸ĳɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="hps" date="2012.7.13   10:42"/>
			bool SetValue(const CString &strParamName, const CString &strParamValue);
			static bool SSetValue(const CString &strParamName, const CString &strParamValue);

		private:
			virtual int OnReflectTo(); 
		};

	}
}