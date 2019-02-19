#include "stdafx.h"
#include "Log.h"
#include <locale.h>

using namespace Ext::File;

std::map<CString, CString> CLog::sm_logNameMapToFile;

CLog::CLog(const CString strRegistLogName)
{
	this->Initial(false, strRegistLogName, _T(""));

	if (CLog::IsRegistLogName(strRegistLogName, this->m_strLogFilePath))
	{
		// ��־������ע��
		this->m_bIsRegisted =  true;
	}
}

CLog::~CLog(void)
{
}

bool CLog::Regist(CString strLogName, CString strLogFilePath)
{
	if (strLogName.IsEmpty() || strLogFilePath.IsEmpty())
		return false;

	//������־�ļ�
	CFileFind find;
	BOOL bRet = find.FindFile(strLogFilePath, 0);

	if (!bRet)
	{
		//��־�ļ������ڣ�������
		CStdioFile fileLog;
		if(fileLog.Open(strLogFilePath, CFile::modeCreate | CFile::modeReadWrite) != TRUE)
		{
			return false;
		}
		fileLog.Close();
	}

	CString file;
	bool hasUsedName = CLog::IsRegistLogName(strLogName, file);
	if (hasUsedName)
	{
		// name - file �Ѿ�ע�����
		if (!file.CompareNoCase(strLogFilePath))
			return true;
		
		// strLogFilePath �Ѿ���������־��ע�ᣬ���ܽ��а�
		CString name;
		bool hasUsedFile = CLog::IsRegistLogFile(strLogFilePath, name);
		if (hasUsedFile)
			return false;
	}
	
	CString name;
	bool hasUsedFile = CLog::IsRegistLogFile(strLogFilePath, name);
	if (hasUsedFile)
		return false;

	// ��ע�����־���ư󶨵���־�ļ�
	sm_logNameMapToFile[strLogName] = strLogFilePath;

	return true;
}

bool CLog::Regist(CString strLogName, CString strLogFilePath, CLog &rLog)
{
	rLog.Initial(false, _T(""), _T(""));

	bool isOk = CLog::Regist(strLogName, strLogFilePath);
	if (!isOk)
		return false;

	rLog.Initial(true, strLogName, strLogFilePath);

	return true;
}

bool CLog::Regist(const CString &strLogFilePath)
{
	bool isOk = CLog::Regist(this->m_strLogName, strLogFilePath);
	if (!isOk)
		return false;

	this->Initial(true, this->m_strLogName, strLogFilePath);
	return true;
}

bool CLog::IsRegistLogName(const CString &strLogName, CString &strLogFilePath)
{
	strLogFilePath = _T("");

	if (sm_logNameMapToFile.count(strLogName) == 1)
	{
		strLogFilePath = sm_logNameMapToFile[strLogName];
		return true;
	}
	return false;
}

bool CLog::IsRegistLogFile(const CString &strLogFilePath, CString &strLogName)
{
	strLogName = _T("");

	std::map<CString, CString>::iterator iter = sm_logNameMapToFile.begin();
	for (; iter != sm_logNameMapToFile.end(); ++iter)
	{
		CString file = iter->second;
		if (!strLogFilePath.CompareNoCase(file))
		{
			strLogName = iter->first;
			return true;
		}
	}

	return false;
}

void CLog::Write(const TCHAR *strMessageFormat, ...)
{
	if (!this->m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!this->GetWritor(writor))
		return;

	TCHAR szBuffer[1024] = _T("");
	
	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	writor.WriteString(szBuffer);
	writor.Close();
}

void CLog::WriteLine(const TCHAR *strMessageFormat, ...)
{
	if (!this->m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!this->GetWritor(writor))
		return;

	TCHAR szBuffer[1024] = _T("");

	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	// д������Ϣ
	writor.WriteString(szBuffer);

	// ����
	writor.WriteString(_T("\n"));

	writor.Close();
}

void CLog::WriteTS(const TCHAR *strMessageFormat, ...)
{
	if (!this->m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!this->GetWritor(writor))
		return;

	// дʱ���
	CTime time = CTime::GetCurrentTime();
	CString strTime = time.Format(_T("[%Y-%m-%d %H:%M:%S] "));
	writor.WriteString(strTime);

	// д��־��Ϣ
	TCHAR szBuffer[1024] = _T("");

	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	writor.WriteString(szBuffer);
	writor.Close();
}

void CLog::WriteLineTS(const TCHAR *strMessageFormat, ...)
{
	if (!this->m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!this->GetWritor(writor))
		return;

	// дʱ���
	CTime time = CTime::GetCurrentTime();
	CString strTime = time.Format(_T("[%Y-%m-%d %H:%M:%S] "));
	writor.WriteString(strTime);

	TCHAR szBuffer[1024] = _T("");
	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	// д��־��Ϣ
	writor.WriteString(szBuffer);

	// ����
	writor.WriteString(_T("\n"));

	writor.Close();
}

CString CLog::GetLogName() const
{
	return this->m_strLogName;
}

CString CLog::GetLogFilePath() const
{
	return this->m_strLogFilePath;
}

bool CLog::GetWritor(CStdioFile &rWritor)
{
	if(rWritor.Open(m_strLogFilePath, CFile::modeReadWrite) != TRUE)
		return false;

	rWritor.SeekToEnd();
	return true;
}

void CLog::Initial(bool bIsRegisted, const CString &strLogName, const CString strLogFilePath)
{
	this->m_bIsRegisted = bIsRegisted;
	this->m_strLogName = strLogName;
	this->m_strLogFilePath = strLogFilePath;
}

bool CLog::IsRegistered()
{
	return this->m_bIsRegisted;
}

bool CLog::UnRegist(CString strLogName)
{
	size_t removeCount = sm_logNameMapToFile.erase(strLogName);
	if (removeCount == 0 || removeCount == 1) // 0 ��ʾ����־���ƻ�δע�ᣬ1 ��ʾ�ɹ��Ƴ��󶨹�ϵ
		return true;

	return false;
}

bool CLog::UnRegist()
{
	return CLog::UnRegist(this->m_strLogName);
}

void CLog::WriteTo(const CString &strLogName, const TCHAR *strMessageFormat, ...)
{
	CLog log(strLogName);
	
	if (!log.m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!log.GetWritor(writor))
		return;

	TCHAR szBuffer[1024] = _T("");

	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	char *old_locale =_strdup(setlocale(LC_CTYPE, NULL));
	setlocale(LC_CTYPE, "chs");//�趨<ctpye.h>���ַ�����ʽ

	writor.WriteString(szBuffer);
	writor.Close();

	free(old_locale);//��ԭ�����趨
}

void CLog::WriteLineTo(const CString &strLogName, const TCHAR *strMessageFormat, ...)
{
	CLog log(strLogName);

	if (!log.m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!log.GetWritor(writor))
		return;

	TCHAR szBuffer[1024] = _T("");

	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	char *old_locale =_strdup(setlocale(LC_CTYPE, NULL));
	setlocale(LC_CTYPE, "chs");//�趨<ctpye.h>���ַ�����ʽ

	writor.WriteString(szBuffer);
	
	// �Ȼ���
	writor.WriteString(_T("\n"));

	writor.Close();

	free(old_locale);//��ԭ�����趨
}

void CLog::WriteTSTo(const CString &strLogName, const TCHAR *strMessageFormat, ...)
{
	CLog log(strLogName);

	if (!log.m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!log.GetWritor(writor))
		return;

	// дʱ���
	CTime time = CTime::GetCurrentTime();
	CString strTime = time.Format(_T("[%Y-%m-%d %H:%M:%S] "));
	writor.WriteString(strTime);

	// д��־��Ϣ
	TCHAR szBuffer[1024] = _T("");

	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	char *old_locale =_strdup(setlocale(LC_CTYPE, NULL));
	setlocale(LC_CTYPE, "chs");//�趨<ctpye.h>���ַ�����ʽ

	writor.WriteString(szBuffer);
	writor.Close();

	free(old_locale);//��ԭ�����趨
}

void CLog::WriteLineTSTo(const CString &strLogName, const TCHAR *strMessageFormat, ...)
{
	CLog log(strLogName);

	if (!log.m_bIsRegisted)
		return;

	CStdioFile writor;
	if (!log.GetWritor(writor))
		return;

	// дʱ���
	CTime time = CTime::GetCurrentTime();
	CString strTime = time.Format(_T("[%Y-%m-%d %H:%M:%S] "));
	writor.WriteString(strTime);

	TCHAR szBuffer[1024] = _T("");
	va_list argList;
	va_start(argList, strMessageFormat);
	_vsntprintf(szBuffer, 1024, strMessageFormat, argList);   
	va_end(argList);

	char *old_locale =_strdup(setlocale(LC_CTYPE, NULL));
	setlocale(LC_CTYPE, "chs");//�趨<ctpye.h>���ַ�����ʽ

	// д��־��Ϣ
	writor.WriteString(szBuffer);

	// ����
	writor.WriteString(_T("\n"));

	writor.Close();

	free(old_locale);//��ԭ�����趨
}