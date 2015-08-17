
// Traffic_Signal_Client_DemoDlg.h : header file
//

#pragma once

#include "ThreadImgProc.h"
// CTraffic_Signal_Client_DemoDlg dialog
class CTraffic_Signal_Client_DemoDlg : public CDialogEx
{
// Construction
public:
	CTraffic_Signal_Client_DemoDlg(CWnd* pParent = NULL);	// standard constructor

// Dialog Data
	enum { IDD = IDD_TRAFFIC_SIGNAL_CLIENT_DEMO_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV support


// Implementation
protected:
	HICON m_hIcon;
	CWinThread* pProcThread;
	ThreadImgProcInfo procThreadInfo;

	CWinThread* pSocketTcpListenThread;
	CWinThread* pSocketUdpListenThread;

	LRESULT OnDisplay(WPARAM wParam, LPARAM lParam);
	// Generated message map functions
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedCaptureOnButton();
	afx_msg void OnBnClickedProcessButton();
	afx_msg void OnBnClickedPauseButton();
	afx_msg void OnBnClickedConnectButton();
private:
	BOOL f_Capture;
	BOOL f_Pause;
	BOOL f_TCP_Connect;
	BOOL f_UDP_Connect;
public:
	afx_msg void OnBnClickedUdpListenButton();
	afx_msg void OnBnClickedGpsInfoButton();
};
