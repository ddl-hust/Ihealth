//********************************************************************
//**	Created:	2011.11.28  14:59
//**	Author:		luzj
//**	
//**	Purpose:	�ַ�����ѯ���������չ��
//**	Remark:		
//********************************************************************
#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

//#include <vector>
#include "List.h"
using namespace Ext::Collection;

namespace Ext
{

	class DCICOMMONEXT_MODULE_EXPIMP CExString
	{
	public:
		/// <summary>
		/// ����ָ���ָ�������ַ���������
		/// </summary>
		/// <remarks></remarks>
		/// <param name="aryResult">�����ֺ�õ����ַ�������</param>
		/// <param name="strOrign">����ԭʼ�ַ���</param>
		/// <param name="strSplitSymbol">���������ַ���(������Ϊ��ֱ�־���ַ���)</param>
		/// <returns>�����������ĳ���</returns>
		/// <author name="Luzj" date="2011.11.28  13:34"/>
		static int SplitStrBySymbol(List<CString> &aryResult,CString strOrign,CString strSplitSymbol=_T(";"));
		//static int SplitStrBySymbol(CStringArray &aryResult,CString strOrign,CString strSplitSymbol=_T(";"));
		//static int SplitStrBySymbol(std::vector<CString> &aryResult,CString strOrign,CString strSplitSymbol=_T(";"));

		/// <summary>
		/// �ж�һ���ַ����������Ƿ����һ���ַ���(�ȳ���)
		/// </summary>
		/// <remarks></remarks>
		/// <param name="arySource">�����ַ�������</param>
		/// <param name="strFind">����Ҫ���ҵ��ַ���</param>
		/// <param name="bMatchCase">�����Ƿ����ִ�Сд,TRUE����,FALSE������</param>
		/// <param name="nFindStart">������ʼ��ѯλ��</param>
		/// <returns>�������,��������λ��(0��ʾ��һλ),�����ڷ���-1</returns>
		/// <author name="Luzj" date="2011.11.28  13:47"/>
		static int FindStrInArray(const CStringArray &arySource,CString strFind,BOOL bMatchCase=TRUE,int nFindStart=0);
		static int FindStrInArray(const List<CString> &arySource,CString strFind,BOOL bMatchCase=TRUE,int nFindStart=0);

		/// <summary>
		/// ȥ���ַ����������ظ����ַ���
		/// </summary>
		/// <remarks></remarks>
		/// <param name="arySource">�������Ҫ������ַ���</param>
		/// <param name="bCompareCase">�����Ƿ��Ǵ�Сд, TRUE���ִ�Сд(ȱʡ), FALSE������</param>
		/// <returns>���ش�������ֵ����</returns>
		/// <author name="Luzj" date="2011.11.28  14:30"/>
		//static int RemoveSameStrInArray(CStringArray &arySource,BOOL bCompareCase=TRUE);
		static int RemoveSameStrInArray(List<CString> &arySource,BOOL bCompareCase=TRUE);

		/// <summary>
		/// ������������ַ���,��ȥ��������ĩβ�����0
		/// </summary>
		/// <remarks></remarks>
		/// <param name="dFormatStr">������Ҫת����doubleֵ</param>
		/// <returns>���ض�Ӧ���ַ���</returns>
		/// <author name="Luzj" date="2011.11.28  14:49"/>
		static CString FormatDouble(double dFormatStr);

		static CString FormatString(CString strText,int nFormat=6);		

		/// <summary>
		/// ��ȡ���������������֡�
		/// </summary>
		/// <remarks></remarks>
		/// <param name="str">[I] ���뺬�и��������ַ�����</param>
		/// <returns>���ر�ʾ�������������ֵ��ַ�����</returns>
		/// <author name="peishengh" date="2011.11.30   16:13"/>
		static CString GetInteger(const CString &str);

		/// <summary>
		/// �� Unicode �ַ�ת��Ϊ ASCII �ַ���
		/// </summary>
		/// <remarks>�����߸����ͷŷ���ֵ����Դ��</remarks>
		/// <param name="tchStr">[I] ����Ҫת�����ַ�ϵ�С�</param>
		/// <returns>���ض�Ӧ�� ASCII �ַ�ϵ�С�</returns>
		/// <author name="hps" date="2012.4.10   15:55"/>
		static char* TCHAR2char(const TCHAR* tchStr);

		/// <summary>
		/// �� ASCII �ַ�ת��Ϊ Unicode �ַ���
		/// </summary>
		/// <remarks>�����߸����ͷŷ���ֵ����Դ��</remarks>
		/// <param name="charStr">[I] ����Ҫת�����ַ�ϵ�С�</param>
		/// <returns>���ض�Ӧ�� Unicode �ַ�ϵ�С�</returns>
		/// <author name="hps" date="2012.4.11   11:35"/>
		static TCHAR* char2TCHAR(const char* charStr);

		static CString C2W(const char* chr);
		static std::string  W2C(const wchar_t* wchr);

		/*****************************************************************************
		* ��һ���ַ���Unicode(UCS-2��UCS-4)����ת����UTF-8����.
		*
		* ����:
		*    unic     �ַ���Unicode����ֵ
		*    pOutput  ָ����������ڴ洢UTF8����ֵ�Ļ�������ָ��
		*    outsize  pOutput����Ĵ�С
		*
		* ����ֵ:
		*    ����ת������ַ���UTF8������ռ���ֽ���, ��������򷵻� 0 .
		*
		* ע��:
		*     1. UTF8û���ֽ�������, ����Unicode���ֽ���Ҫ��;
		*        �ֽ����Ϊ���(Big Endian)��С��(Little Endian)����;
		*        ��Intel�������в���С�˷���ʾ, �ڴ˲���С�˷���ʾ. (�͵�ַ���λ)
		*     2. �뱣֤ pOutput �������������� 6 �ֽڵĿռ��С!
		****************************************************************************/
		static int	  OneUnicoide2UTF8(unsigned long unic, unsigned char *pOutput);

		/*****************************************************************************
		* ��һ���ַ���UTF8����ת����Unicode(UCS-2��UCS-4)����.
		*
		* ����:
		*    pInput      ָ�����뻺����, ��UTF-8����
		*	  utfbytes    utf8�ַ���λ��
		*    Unic        ָ�����������, �䱣������ݼ���Unicode����ֵ,
		*                ����Ϊunsigned long .
		*
		* ����ֵ:
		*    �ɹ��򷵻ظ��ַ���UTF8������ռ�õ��ֽ���; ʧ���򷵻�0.
		*
		* ע��:
		*     1. UTF8û���ֽ�������, ����Unicode���ֽ���Ҫ��;
		*        �ֽ����Ϊ���(Big Endian)��С��(Little Endian)����;
		*        ��Intel�������в���С�˷���ʾ, �ڴ˲���С�˷���ʾ. (�͵�ַ���λ)
		****************************************************************************/
		static int	  OneUTF82Unicode(const unsigned char *pInput, int utfbytes, unsigned long *Unic);

		/// <summary>
		/// �� Unicode �ַ�ת��Ϊ UTF8 �ַ���
		/// </summary>
		/// <remarks>�����߸��������ڴ�ռ䡣</remarks>
		/// <param name="pInput">[I] ����Ҫת�����ַ����С�</param>
		/// <param name="inLength">[I] ָʾ pInput �ĳ��ȡ�</param>
		/// <param name="pOutPut">[O] ���ת������ַ����顣</param>
		/// <param name="outLength">[O] ָʾ pOutPut �ĳ��ȡ�</param>
		/// <returns>����ʵ��ת�����ַ�����</returns>
		/// <author name="quanl" date="2013.6.27   11:35"/>
		static size_t Unicode2Utf8(const wchar_t *pInput, size_t inLength, char *pOutPut, size_t outLength);

		/// <summary>
		/// �� UTF8 �ַ�ת��Ϊ Unicode �ַ���
		/// </summary>
		/// <remarks>�����߸��������ڴ�ռ䡣</remarks>
		/// <param name="pInput">[I] ����Ҫת�����ַ����С�</param>
		/// <param name="inLength">[I] ָʾ pInput �ĳ��ȡ�</param>
		/// <param name="pOutPut">[O] ���ת������ַ����顣</param>
		/// <param name="outLength">[O] ָʾ out �ĳ��ȡ�</param>
		/// <returns>����ʵ��ת�����ַ�����</returns>
		/// <author name="quanl" date="2013.6.27   11:35"/>
		static size_t Utf82Unicode(char *pInput, size_t insize, wchar_t *pOutput,  size_t outsize);

		//���һ���ַ��������ַ���������������ɵģ��Ѹ��ַ�����ֳ��ַ�������������
		//�ο�˵��  : ��������������ַ�����Ϊ: �ַ������� �� ���֣��ַ���
		static BOOL SplitStrInTwoPart(CString &strStr,	//�������:��ֺ���ַ�������
			CString &strDigit,							//�������:��ֺ�����ֲ���
			CString strOrign);							//�������:ԭʼ�ַ���

		//���һ���ַ����Ƿ������ִ�
		//�ο�˵��	: ���ַ��������С������ѧ����������ʽ����Ϊ������
		static BOOL IsDigit(CString strStr);		//�������:Դ�ַ���				

	private:
		CExString(void){};
		~CExString(void){};
	};

} // End namespace Ext