#pragma once

const TCHAR kWidgetSlider[] =	 _T("passive_train_voice");

enum WidgetType				 //ѡ��С�Ҽ�������
{
	CHANGE_LIST_TRANS,		//�ı�������Ĳ����б�����
	CHANGE_MAINWND_TRANS,	//�ı�������Ƥ����͸����
	CHANGE_VOLUME,			//�ı������������������Ǹ��Ժ��õ�	
	UNKNOWN,
};


class CPopupWidget:  public CWindowWnd, public INotifyUI
{
public:
    explicit CPopupWidget();

protected:
    virtual ~CPopupWidget();  

public:
	LPCTSTR GetWindowClassName() const;
	UINT GetClassStyle() const;
	void OnFinalMessage(HWND /*hWnd*/);

	void InitWindow();
	void Init(LPCTSTR pszXMLPath, HWND hWndParent, CPaintManagerUI* pMainPaintManager, POINT ptPos, WidgetType type);
	void Notify(TNotifyUI& msg);

	LRESULT OnCreate(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnClose(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnDestroy(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnNcActivate(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnNcCalcSize(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnNcPaint(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnNcHitTest(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnKillFocus(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT OnMouseLeave(UINT uMsg, WPARAM wParam, LPARAM lParam, BOOL& bHandled);
	LRESULT HandleMessage(UINT uMsg, WPARAM wParam, LPARAM lParam);
   
protected:
	CDuiString		m_strXMLPath;
	CPaintManagerUI *m_MainWndpm;
	CPaintManagerUI m_pm;
	CButtonUI		*m_pBtnParent;
	CSliderUI		*m_pSlider;
	WidgetType		m_WidgetType;
};


int GetSystemVolume();
void SetSystemVolume(int nValue);

int GetSystemVolumeWin7();
void SetSystemVolumeWin7(int nVolume);