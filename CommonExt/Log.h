#pragma once

#ifdef DCICOMMONEXT_MODULE
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllexport)
#else
#define DCICOMMONEXT_MODULE_EXPIMP __declspec(dllimport)
#endif

#include <map>

// ע�����־���ƣ���һ����־�ļ����а�
#define DCI_LOG _T("DCI_LOG")

namespace Ext
{
	namespace File
	{

		/// <summary>
		/// ��ʾ��־��
		/// </summary>
		class DCICOMMONEXT_MODULE_EXPIMP CLog
		{
		public:

			/// <summary>
			/// ʹ��ע�����־���Ƴ�ʼ��һ����־���ʵ����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strRegistLogName">[I] ָ��һ���Ѿ�ע�����־���ơ�</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.13   17:31"/>
			CLog(const CString strRegistLogName);

			/// <summary>
			/// ����������
			/// </summary>
			/// <remarks></remarks>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.13   17:32"/>
			virtual ~CLog(void);

			/// <summary>
			/// ע��һ����־���ơ�
			/// </summary>
			/// <remarks>
			/// <br> 1. һ����־���ƺ�һ���ļ�ֻ�ܴ���һ��һ�Ĺ�ϵ��</br>
			/// <br> 2. ���һ����־�����Ѿ���ע�ᣬ��ָ���ԭ����ͬ����־�ļ�ʱ��</br>
			/// <br>    �����µ���־�ļ�û�б��󶨵�������־���ƲŻ�ע��ɹ���</br>
			/// <br>    ���Ҿɵİ󶨹�ϵ��ɾ����</br>
			/// </remarks>
			/// <param name="strLogName">[I] ָ��Ҫ����ע�����־���ơ�</param>
			/// <param name="strLogFilePath">[I] ָ����Ҫ�󶨵���־�ļ���</param>
			/// <returns>��ע��ɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="peishengh" date="2011.10.13   17:32"/>
			static bool Regist(CString strLogName, CString strLogFilePath);

			/// <summary>
			/// ע��һ����־���ơ�
			/// </summary>
			/// <remarks>
			/// <br> 1. һ����־���ƺ�һ���ļ�ֻ�ܴ���һ��һ�Ĺ�ϵ��</br>
			/// <br> 2. ���һ����־�����Ѿ���ע�ᣬ��ָ���ԭ����ͬ����־�ļ�ʱ��</br>
			/// <br>    �����µ���־�ļ�û�б��󶨵�������־���ƲŻ�ע��ɹ���</br>
			/// <br>    ���Ҿɵİ󶨹�ϵ��ɾ����</br>
			/// </remarks>
			/// <param name="strLogName">[I] ָ��Ҫ����ע�����־���ơ�</param>
			/// <param name="strLogFilePath">[I] ָ����Ҫ�󶨵���־�ļ���</param>
			/// <param name="rLog">[O] ����ע��ɹ��� CLog ʵ����</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.14   12:32"/>
			static bool Regist(CString strLogName, CString strLogFilePath, CLog &rLog);

			/// <summary>
			/// ���һ���Ѿ�ע�����־���ơ�
			/// </summary>
			/// <remarks>�������־���ƺ���־�ļ��İ󶨹�ϵ��</remarks>
			/// <param name="strLogName">[I] ָ��Ҫ�������־���ơ�</param>
			/// <returns>������ɹ����򷵻� true�����򷵻� false����ָ������־����δע�ᣬ�򷵻� true��</returns>
			/// <author name="peishengh" date="2011.12.7   16:57"/>
			static bool UnRegist(CString strLogName);

			/// <summary>
			/// ��ȡһ��ֵ��ָʾָ������־�����Ƿ��Ѿ���ע�ᡣ
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strLogName">[I] ָ��Ҫ������־����</param>
			/// <param name="strLogFilePath">[O] ����ָ������־���󶨵���־�ļ���������־����δ��ע�ᣬ���ļ���Ϊ�ա�</param>
			/// <returns>������־���ѱ�ע�ᣬ�򷵻� true�����򷵻� false��</returns>
			/// <author name="peishengh" date="2011.10.13   17:34"/>
			static bool IsRegistLogName(const CString &strLogName, CString &strLogFilePath);

			/// <summary>
			/// ��ȡһ��ֵ��ָʾָ�����ļ��Ƿ��Ѿ����󶨵�һ����־����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strLogFilePath">[I] ָ��Ҫ���</param>
			/// <param name="strLogName"></param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.13   17:37"/>
			static bool IsRegistLogFile(const CString &strLogFilePath, CString &strLogName);

			/// <summary>
			/// ��ǰ��־ע�ᵽָ���ļ�·����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strLogFilePath">[I] ָ����ǰ��־Ҫ�󶨵����ļ�·����</param>
			/// <returns>��ע��ɹ����򷵻� true�����򷵻� false��</returns>
			/// <author name="peishengh" date="2011.10.14   12:26"/>
			bool Regist(const CString &strLogFilePath);

			/// <summary>
			/// <br> д��־��Ϣ��</br>
			/// <br> ����ǰ��־����δע�ᵽ�κ���־�ļ����򲻽����κβ�����</br>
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strMessageFormat">[I] ָ��Ҫд����־�ļ�����Ϣ��</param>
			/// <param name="...">[I] ָ��һ��������ʽ��������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.13   17:53"/>
			void Write(const TCHAR *strMessageFormat, ...);
			static void WriteTo(const CString &strLogName, const TCHAR *strMessageFormat, ...);

			/// <summary>
			/// <br> ��������дһ����־��Ϣ��</br>
			/// <br> ����ǰ��־����δע�ᵽ�κ���־�ļ����򲻽����κβ�����</br>
			/// </summary>
			/// <remarks></remarks>
			/// <param name="strMessageFormat">[I] ָ��Ҫд����־�ļ�����Ϣ��</param>
			/// <param name="...">[I] ָ��һ��������ʽ��������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.13   17:54"/>
			void WriteLine(const TCHAR *strMessageFormat, ...);
			static void WriteLineTo(const CString &strLogName, const TCHAR *strMessageFormat, ...);

			/// <summary>
			/// <br> д��־��Ϣ��</br>
			/// <br> ����ǰ��־����δע�ᵽ�κ���־�ļ����򲻽����κβ�����</br>
			/// </summary>
			/// <remarks>�˷�������д��־��Ϣ֮ǰ����׷��һ��ʱ�����Time Stamp�� [Y.M.D H:M:S]</remarks>
			/// <param name="strMessageFormat">[I] ָ��Ҫд����־�ļ�����Ϣ��</param>
			/// <param name="">[I] ָ��һ��������ʽ��������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.13   17:55"/>
			void WriteTS(const TCHAR *strMessageFormat, ...);
			static void WriteTSTo(const CString &strLogName, const TCHAR *strMessageFormat, ...);

			/// <summary>
			/// <br> ��������дһ����־��Ϣ��</br>
			/// <br> ����ǰ��־����δע�ᵽ�κ���־�ļ����򲻽����κβ�����</br>
			/// </summary>
			/// <remarks>�˷�������д��־��Ϣ֮ǰ����׷��һ��ʱ�����Time Stamp�� [Y.M.D H:M:S]</remarks>
			/// <param name="strMessageFormat">[I] ָ��Ҫд����־�ļ�����Ϣ��</param>
			/// <param name="">[I] ָ��һ��������ʽ��������</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.13   17:57"/>
			void WriteLineTS(const TCHAR *strMessageFormat, ...);
			static void WriteLineTSTo(const CString &strLogName, const TCHAR *strMessageFormat, ...);

			/// <summary>
			/// ��ȡ��ǰ��־���ơ�
			/// </summary>
			/// <remarks></remarks>
			/// <returns>���ص�ǰ��־���ơ�</returns>
			/// <author name="peishengh" date="2011.10.14   11:39"/>
			CString GetLogName() const;

			/// <summary>
			/// ��ȡ��ǰ��־��Ӧ����־�ļ���
			/// </summary>
			/// <remarks></remarks>
			/// <returns>���ص�ǰ�󶨵�����־�ļ���</returns>
			/// <author name="peishengh" date="2011.10.13   17:58"/>
			CString GetLogFilePath() const;

			/// <summary>
			/// ��ȡһ��ֵ������ָʾ��ǰ��־�����Ƿ�ע���ˡ�
			/// </summary>
			/// <remarks>�ɵ��� CLog::Regist(...) ��̬����ע��һ����־���ơ�</remarks>
			/// <returns>�����ǰ��־�����Ѿ�ע����ˣ��򷵻� true�����򷵻� false��</returns>
			/// <author name="peishengh" date="2011.10.13   17:59"/>
			bool IsRegistered();

			/// <summary>
			/// �����ǰ��־���Ƶ�ע���ļ���
			/// </summary>
			/// <remarks>>�������־���ƺ���־�ļ��İ󶨹�ϵ��</remarks>
			/// <returns>������ɹ����򷵻� true�����򷵻� false������ǰ��־����δע�ᣬ�򷵻� true��</returns>
			/// <author name="peishengh" date="2011.12.7   17:05"/>
			bool UnRegist();

		private:

			/// <summary>
			/// ��ȡд�ļ��� StdioFile ����
			/// </summary>
			/// <remarks>��д����ʽ���ļ�������д����Ƶ��ļ�ĩβ��</remarks>
			/// <param name="rWritor"></param>
			/// <returns>���ɹ���ʼ�� StdioFile �����򷵻� true�����򷵻� false��</returns>
			/// <author name="peishengh" date="2011.10.13   17:59"/>
			bool GetWritor(CStdioFile &rWritor);

			/// <summary>
			/// ��ʼ����ǰ����
			/// </summary>
			/// <remarks></remarks>
			/// <param name="bIsRegisted">[I] ��ʾָ����־���Ƿ�ע����û�С�</param>
			/// <param name="strLogName">[I] ָ����־���ơ�</param>
			/// <param name="strLogFilePath">[I] ָ����־�ļ�·����</param>
			/// <returns></returns>
			/// <author name="peishengh" date="2011.10.14   12:23"/>
			void Initial(bool bIsRegisted, const CString &strLogName, const CString strLogFilePath);

		private:
			/// <summary>
			/// ��־�ļ���
			/// </summary>
			CString m_strLogName;

			/// <summary>
			/// ��־�ļ��󶨵�����־�ļ�·��
			/// </summary>
			CString m_strLogFilePath;

			/// <summary>
			/// ָʾ��ǰ����־���Ƿ�ע����û��
			/// </summary>
			bool m_bIsRegisted;

			/// <summary>
			/// �����Ѿ�ע�����־�ļ�������ʽ��<��־�ļ���, �󶨵�����־�ļ�>
			/// </summary>
			static std::map<CString, CString> sm_logNameMapToFile;
		};

	}// End namespace File
}// End namespace Ext