
// InVehicle_EmulatorDlg.h : header file
//

#pragma once

#include "selfDefineMessage.h"
// CInVehicle_EmulatorDlg dialog
class CInVehicle_EmulatorDlg : public CDialogEx
{
// Construction
public:
	CInVehicle_EmulatorDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_INVEHICLE_EMULATOR_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;
	
	//for self define message
	ThreadProcInfo procThreadInfo;
	LRESULT OnSendCounterDisplay(WPARAM wParam, LPARAM lParam);
	LRESULT OnStatusDisplay(WPARAM wParam, LPARAM lParam);
	//for self define function
	void appInitEvents(void);
	int appStartImplement(ThreadProcInfo *ProcInfoParam);
	bool startSocket();


	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnClickedBrowseButton();
	CString m_strFilePath;
	afx_msg void OnClickedStartButton();
	afx_msg void OnClickedPauseButton();
	afx_msg void OnClickedStopButton();
	CIPAddressCtrl m_ServerIp;
	CString m_port;
	CButton m_startbutton;
	CButton m_stopbutton;
	CButton m_pausebutton;
	CString m_timedelay;
	//add a font class
	CFont font;
};
