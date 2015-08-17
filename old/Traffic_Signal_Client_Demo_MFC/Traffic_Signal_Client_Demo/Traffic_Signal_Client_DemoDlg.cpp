
// Traffic_Signal_Client_DemoDlg.cpp : implementation file
//

#include "stdafx.h"
#include "Traffic_Signal_Client_Demo.h"
#include "Traffic_Signal_Client_DemoDlg.h"
#include "afxdialogex.h"
#include "WinSock2.h"
#include <afxsock.h>  

#include "ThreadImgProc.h"
#include "ThreadSocketClientTCP.h"
#include "ThreadSocketServerUDP.h"
#include "GPS_NEMA_Data.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CAboutDlg dialog used for App About

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// Dialog Data
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

// Implementation
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CTraffic_Signal_Client_DemoDlg dialog




CTraffic_Signal_Client_DemoDlg::CTraffic_Signal_Client_DemoDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CTraffic_Signal_Client_DemoDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
    f_Capture = FALSE;
	f_Pause   = FALSE;
	f_TCP_Connect = FALSE;
	f_UDP_Connect = FALSE;
}

void CTraffic_Signal_Client_DemoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CTraffic_Signal_Client_DemoDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_MESSAGE(WM_DISPLAY, OnDisplay)
	ON_BN_CLICKED(IDC_CAPTURE_ON_BUTTON, &CTraffic_Signal_Client_DemoDlg::OnBnClickedCaptureOnButton)
	ON_BN_CLICKED(IDC_PAUSE_BUTTON, &CTraffic_Signal_Client_DemoDlg::OnBnClickedPauseButton)
	ON_BN_CLICKED(IDC_CONNECT_BUTTON, &CTraffic_Signal_Client_DemoDlg::OnBnClickedConnectButton)
	ON_BN_CLICKED(IDC_UDP_LISTEN_BUTTON, &CTraffic_Signal_Client_DemoDlg::OnBnClickedUdpListenButton)
	ON_BN_CLICKED(IDC_GPS_INFO_BUTTON, &CTraffic_Signal_Client_DemoDlg::OnBnClickedGpsInfoButton)
END_MESSAGE_MAP()


// CTraffic_Signal_Client_DemoDlg message handlers

BOOL CTraffic_Signal_Client_DemoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon

	// TODO: Add extra initialization here
	AfxSocketInit();

	PHOSTENT hostinfo;
	char gcTemp[255];
	char *localIP;
	if( 0 == gethostname(gcTemp,255) )
	{
		hostinfo = gethostbyname(gcTemp);
		localIP  = inet_ntoa(*(struct in_addr *)*hostinfo->h_addr_list);
		CString tempStr;
		tempStr.Format(localIP);

		SetDlgItemText(IDC_LOCAL_IP_STATIC,tempStr);
	}

	//set default value of UDP port
	SetDlgItemText(IDC_LOCAL_PORT_EDIT,"100");

	//get current window rect
	//CRect oldrect;
	//GetWindowRect(&oldrect);

	//set default window rect
	CRect temprect(0,0,820,600);

	CWnd::SetWindowPos(NULL,0,0,temprect.Width(),temprect.Height(),SWP_NOZORDER|SWP_NOMOVE);

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CTraffic_Signal_Client_DemoDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CTraffic_Signal_Client_DemoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // device context for painting

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// Center icon in client rectangle
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// Draw the icon
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// The system calls this function to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CTraffic_Signal_Client_DemoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CTraffic_Signal_Client_DemoDlg::OnBnClickedCaptureOnButton()
{
	// TODO: Add your control notification handler code here
		
    if(f_Capture)
	{
		f_Capture = FALSE;
		//SetDlgItemText(IDC_CAPTURE_ON_BUTTON, "Capture ON");
		if(!sharedMem_Img_UI.CloseCameraDevice())
		{
			//close failed
			return;
		}
		GetDlgItem(IDC_CAPTURE_ON_BUTTON)->SetWindowText("Capture ON");
		
		//to make sure the paused image process thread can exit normally.
		eventPause.SetEvent();
		f_Pause = FALSE;
		SetDlgItemText(IDC_PAUSE_BUTTON, "Pause");
	}else
	{
		f_Capture = TRUE;
		if(!sharedMem_Img_UI.OpenCameraDevice())
		{
			//open failed
			return;
		}
	    
		//set all the window's Rect
		CRect rect1,rect2,rect3,rect4;
		GetDlgItem(IDC_MAIN_WINDOW_PIC)->GetClientRect(&rect1);
		GetDlgItem(IDC_UP_PIC)->GetClientRect(&rect2);
		GetDlgItem(IDC_CENTER_PIC)->GetClientRect(&rect3);
		GetDlgItem(IDC_DOWN_PIC)->GetClientRect(&rect4);
		sharedMem_Img_UI.setPicCRect(rect1,rect2,rect3,rect4);

		procThreadInfo.hWnd = this->GetSafeHwnd();

	    pProcThread=AfxBeginThread(ThreadImgProc,
		(LPVOID)&procThreadInfo,
		THREAD_PRIORITY_NORMAL,
		0,
		CREATE_SUSPENDED);
	
		pProcThread->ResumeThread();
		eventPause.SetEvent();

		//SetDlgItemText(IDC_CAPTURE_ON_BUTTON, "Capture OFF");
		GetDlgItem(IDC_CAPTURE_ON_BUTTON)->SetWindowText("Capture OFF");
	}
}

void CTraffic_Signal_Client_DemoDlg::OnBnClickedPauseButton()
{
	// TODO: Add your control notification handler code here
	if(f_Pause)
	{
		eventPause.SetEvent();
		f_Pause = FALSE;
		SetDlgItemText(IDC_PAUSE_BUTTON, "Pause");
	}else
	{
		eventPause.ResetEvent();
		f_Pause = TRUE;
		SetDlgItemText(IDC_PAUSE_BUTTON, "Restart");
	}
}


void CTraffic_Signal_Client_DemoDlg::OnBnClickedConnectButton()
{
	// TODO: Add your control notification handler code here
	if(f_TCP_Connect)
	{
		f_TCP_Connect = FALSE;
		SetDlgItemText(IDC_CONNECT_BUTTON, "Connect");
		closesocket(g_ClientSockTCP);
		
	}else
	{

		DWORD sourceIP;
	    ((CIPAddressCtrl*)GetDlgItem(IDC_SERVER_IPADDRESS))->GetAddress(sourceIP);

		g_ClientSockTCP = socket(AF_INET,SOCK_STREAM, IPPROTO_TCP);

		if(NULL == g_ClientSockTCP)
		{
			MessageBox(_T("create socket failed!"));
			return ;
		}

		sockaddr_in addrSock;
		addrSock.sin_family = AF_INET;
		addrSock.sin_port = htons(GetDlgItemInt(IDC_SERVER_PORT_EDIT));
		addrSock.sin_addr.S_un.S_addr = htonl(sourceIP);

		if(SOCKET_ERROR == connect(g_ClientSockTCP,(sockaddr *)&addrSock, sizeof(addrSock)))
		{
			MessageBox(_T("connect failed, pls input correct IP and port"));
			//(Static_text)->SetWindowTextW(ss);
			return ;
		}

		//create a thread to listen the data from server using TCP connect
	    pSocketTcpListenThread=AfxBeginThread(ThreadTCPSocketListen,
		(LPVOID)NULL,
		THREAD_PRIORITY_NORMAL,
		0,
		CREATE_SUSPENDED);
	
		pSocketTcpListenThread->ResumeThread();

		f_TCP_Connect = TRUE;
		SetDlgItemText(IDC_CONNECT_BUTTON, "Disconnect");
	}
}

LRESULT CTraffic_Signal_Client_DemoDlg::OnDisplay(WPARAM wParam, LPARAM lParam)
{
	//display main window picture
	//get the main picture
	CImage *mainPic = sharedMem_Img_UI.ReadCImage1();

	if(NULL != mainPic)
	{
		//get the main window
		CDC *pDC = GetDlgItem(IDC_MAIN_WINDOW_PIC)->GetDC();
		HDC hDC  = pDC->GetSafeHdc();
		CRect rect;
		GetDlgItem(IDC_MAIN_WINDOW_PIC)->GetClientRect(&rect);
		//mainPic->StretchBlt(pDC->m_hDC, rect, SRCCOPY);
        mainPic->Draw(pDC->m_hDC,0,0);
		ReleaseDC(pDC);
	}

    //display right up window picture
	//get the right up picture
	CImage *upPic = sharedMem_Img_UI.ReadCImage2();

	if(NULL != upPic)
	{
		//get the up window
		CDC *pDC = GetDlgItem(IDC_UP_PIC)->GetDC();
		HDC hDC  = pDC->GetSafeHdc();
		CRect rect;
		GetDlgItem(IDC_UP_PIC)->GetClientRect(&rect);
		//upPic->StretchBlt(pDC->m_hDC, rect, SRCCOPY);
        upPic->Draw(pDC->m_hDC,0,0);
		ReleaseDC(pDC);
	}

	//display right center window picture
	//get the right center picture
	CImage *centerPic = sharedMem_Img_UI.ReadCImage3();

	if(NULL != centerPic)
	{
		//get the up window
		CDC *pDC = GetDlgItem(IDC_CENTER_PIC)->GetDC();
		HDC hDC  = pDC->GetSafeHdc();
		CRect rect;
		GetDlgItem(IDC_CENTER_PIC)->GetClientRect(&rect);
		//centerPic->StretchBlt(pDC->m_hDC, rect, SRCCOPY);
        centerPic->Draw(pDC->m_hDC,0,0);
		ReleaseDC(pDC);
	}

	//display right down window picture
	//get the right down picture
	CImage *downPic = sharedMem_Img_UI.ReadCImage4();

	if(NULL != downPic)
	{
		//get the up window
		CDC *pDC = GetDlgItem(IDC_DOWN_PIC)->GetDC();
		HDC hDC  = pDC->GetSafeHdc();
		CRect rect;
		GetDlgItem(IDC_DOWN_PIC)->GetClientRect(&rect);
		//downPic->StretchBlt(pDC->m_hDC, rect, SRCCOPY);
        downPic->Draw(pDC->m_hDC,0,0);
		ReleaseDC(pDC);
	}

	return 0;

	
}

void CTraffic_Signal_Client_DemoDlg::OnBnClickedUdpListenButton()
{
	// TODO: Add your control notification handler code here
    if(f_UDP_Connect)
	{
		f_UDP_Connect = FALSE;
		SetDlgItemText(IDC_UDP_LISTEN_BUTTON, "Listen");
		closesocket(g_ServerSockUDP);
		
	}else
	{
		//create udp socket
		if((g_ServerSockUDP = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == INVALID_SOCKET)
		{
			MessageBox(_T("create UDP socket failed!"));
			return ;
		}

		//bind

		sockaddr_in addrSock;
		addrSock.sin_family = AF_INET;
		addrSock.sin_port = htons(GetDlgItemInt(IDC_LOCAL_PORT_EDIT));
		addrSock.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

		if( SOCKET_ERROR == bind(g_ServerSockUDP, (sockaddr *)&addrSock, sizeof(addrSock)))
		{
			MessageBox(_T("UDP bind failed!"));
			return ;
		}

		pSocketUdpListenThread=AfxBeginThread(ThreadUCPSocketListen,
			(LPVOID)NULL,
			THREAD_PRIORITY_NORMAL,
			0,
			CREATE_SUSPENDED);
	
			pSocketUdpListenThread->ResumeThread();

		f_UDP_Connect = TRUE;
		SetDlgItemText(IDC_UDP_LISTEN_BUTTON, "Stop Listen");
	}

}


void CTraffic_Signal_Client_DemoDlg::OnBnClickedGpsInfoButton()
{
	// TODO: Add your control notification handler code here
	GPGGA_Info gpsData;
	g_GPSInfo.getGPGGAInfo(gpsData);
	
	CString clon;
	CString clat;
	CString calt;
	CString temp;

	temp.Format("%f",gpsData.dLon);
	clon += "Longtitude:";
	clon += temp;
	clon += " ";
	clon += gpsData.cLon;

	temp.Format("%f",gpsData.dLat);
	clat += "Latitude:";
	clat += temp;
	clat += " ";
	clat += gpsData.cLat;

	temp.Format("%f",gpsData.altitude);
	calt += "Altitude:";
	calt += temp;
	calt += " ";
	calt += "M";

	SetDlgItemText(IDC_LON_STATIC, clon);
	SetDlgItemText(IDC_LAT_STATIC, clat);
	SetDlgItemText(IDC_ALTI_STATIC, calt);
}
