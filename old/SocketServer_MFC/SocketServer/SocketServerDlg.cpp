
// SocketServerDlg.cpp : implementation file
//

#include "stdafx.h"
#include "SocketServer.h"
#include "SocketServerDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

CWinThread *pThread = NULL;
SOCKET m_ServerSock;
SOCKET m_ClientSock;
sockaddr_in serverSockaddr;

unsigned int ListenThread(LPVOID lParam)
{
	CSocketServerDlg *aDlg = (CSocketServerDlg *)lParam;
	for(;;)
	{
		char recvData[1000] = {0};
		char outputMsg[1000] = {0};
		int iRead = recv(m_ClientSock, recvData, 1000, 0);
		if(iRead >0)
		{
			//do data base write process
			CString strText;
			aDlg->GetDlgItemText(IDC_OUTPUT_EDIT, strText);
			sprintf(outputMsg, "%s \r\n", recvData);
			strText += outputMsg;
			aDlg->SetDlgItemText(IDC_OUTPUT_EDIT, strText);

			CEdit *outputEdit = (CEdit*)(aDlg->GetDlgItem(IDC_OUTPUT_EDIT));
			outputEdit->LineScroll(outputEdit->GetLineCount());

			//data process

			//send back

			if(SOCKET_ERROR == send(m_ClientSock, recvData, iRead, 0))
			{
				closesocket(m_ClientSock);
				return -2;
			}


		}
		else
		{
			closesocket(m_ClientSock);
			return -1;
		}
	}
	return 0;
}

unsigned int StartServer(LPVOID lParam)
{
    WSADATA wsaData;
	
	WORD wVersionRequested = MAKEWORD(1, 1);
	int nResult = WSAStartup(wVersionRequested, &wsaData);
	if (nResult != 0)
	{
		return 1;
	}

	CSocketServerDlg *aDlg = (CSocketServerDlg *)lParam;

	memset(&serverSockaddr, 0, sizeof(serverSockaddr));

	m_ClientSock = NULL;

	m_ServerSock = socket(AF_INET,SOCK_STREAM, 0);
	if(NULL == m_ServerSock)
	{
		AfxMessageBox(_T("create socket failed!"));
		return 1;
	}

	sockaddr_in addrSock;
	addrSock.sin_family = AF_INET;
	addrSock.sin_port = htons(aDlg->GetDlgItemInt(IDC_PORT_EDIT));
	addrSock.sin_addr.S_un.S_addr = htonl(INADDR_ANY);

	int i = 0;

	if(SOCKET_ERROR == bind(m_ServerSock,(sockaddr *)&addrSock, sizeof(addrSock)))
	{
		AfxMessageBox(_T("Bind Failed"));
		return 1;
	}

	if(SOCKET_ERROR == listen(m_ServerSock,10))
	{
		AfxMessageBox(_T("Listen Failed"));
		return 1;
	}

	CString strText;

	aDlg->GetDlgItemText(IDC_OUTPUT_EDIT, strText);

	strText += "Server Start! \r\n";

	aDlg->SetDlgItemText(IDC_OUTPUT_EDIT, strText);

	while (TRUE)
	{
		//if( Select(m_ServerSock, 100, TRUE) )
		int iLen = sizeof(sockaddr_in);
		m_ClientSock = accept(m_ServerSock, (sockaddr *)&serverSockaddr, &iLen );
		if(INVALID_SOCKET == m_ClientSock)
		{
			continue;
		}

		aDlg->GetDlgItemText(IDC_OUTPUT_EDIT, strText);

		strText += "One socket accepted! \r\n";

		aDlg->SetDlgItemText(IDC_OUTPUT_EDIT, strText);

		CEdit *outputEdit = (CEdit*)(aDlg->GetDlgItem(IDC_OUTPUT_EDIT));
		outputEdit->LineScroll(outputEdit->GetLineCount());

		pThread=AfxBeginThread(ListenThread,
		(LPVOID)lParam,
		THREAD_PRIORITY_NORMAL,
		0,
		CREATE_SUSPENDED);
	
	pThread->ResumeThread();

	}

	closesocket(m_ServerSock);
	aDlg->GetDlgItem(IDC_LISTEN_BUTTON)->EnableWindow(FALSE);
	return 0;
}


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


// CSocketServerDlg dialog




CSocketServerDlg::CSocketServerDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CSocketServerDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CSocketServerDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CSocketServerDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_LISTEN_BUTTON, &CSocketServerDlg::OnBnClickedListenButton)
END_MESSAGE_MAP()


// CSocketServerDlg message handlers

BOOL CSocketServerDlg::OnInitDialog()
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

	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CSocketServerDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CSocketServerDlg::OnPaint()
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
HCURSOR CSocketServerDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CSocketServerDlg::OnBnClickedListenButton()
{
	// TODO: Add your control notification handler code here
	pThread=AfxBeginThread(StartServer,
		(LPVOID)this,
		THREAD_PRIORITY_NORMAL,
		0,
		CREATE_SUSPENDED);
	
	pThread->ResumeThread();

	GetDlgItem(IDC_LISTEN_BUTTON)->EnableWindow(FALSE);

}
