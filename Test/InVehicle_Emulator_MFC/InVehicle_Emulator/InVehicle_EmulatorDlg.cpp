
// InVehicle_EmulatorDlg.cpp : implementation file
//
#include "afxmt.h"
#include "stdafx.h"
#include "InVehicle_Emulator.h"
#include "InVehicle_EmulatorDlg.h"
#include "afxdialogex.h"

/*********************************************************************application define********************************************************/
#include "typeDefine.h"
#include "messageProcessClass.h"
#include "selfDefineMessage.h"
#include "Shlwapi.h"

#pragma comment(lib,"Shlwapi.lib") //for check file exist
#define FILE_PATH_LENGTH 1000

SOCKET sockClient;
SOCKADDR_IN serverAddr;
char dataFileName[FILE_PATH_LENGTH];
HANDLE socketMutex;
HANDLE g_readyEvent_ConnectSocket;
//for thread control
HANDLE g_readyEvent_Send; //for pause control
CCriticalSection critical_section;
volatile char exit_flag =  FALSE;
volatile char pause_flag =  FALSE;

#define RECV_BUF_LEN  10000
uint8 recvBuf[RECV_BUF_LEN];
volatile int timeDelay = 3000;

HANDLE threadHandle[3];


#ifdef _DEBUG
#define new DEBUG_NEW
#endif


void trySetConnectSocket(bool flag)
{
	DWORD dwWaitResult = WaitForSingleObject(socketMutex,0);
	if (dwWaitResult != WAIT_OBJECT_0 && dwWaitResult != WAIT_TIMEOUT)
	{

	}else
	{
		//ReleaseSemaphore(g_readySema_ConnectSocket, 1 ,NULL);
		SetEvent(g_readyEvent_ConnectSocket);
		ReleaseMutex(socketMutex);
	}
}

//unsigned int __stdcall Thread_ReconnectSocket(void *data)
unsigned int __stdcall Thread_ReconnectSocket(LPVOID lpParam)
{
	ThreadProcInfo* pInfo =  (ThreadProcInfo*) lpParam;

	while(1)
	{
		
		WaitForSingleObject(g_readyEvent_ConnectSocket, INFINITE);
		WaitForSingleObject(socketMutex,INFINITE);
		critical_section.Lock();
		if(exit_flag)
		{
			//shutdown(sockClient,SD_BOTH);
			//closesocket(sockClient);	
			critical_section.Unlock();
			ResetEvent(g_readyEvent_ConnectSocket);
			ReleaseMutex(socketMutex);
			return -1;
		}
		critical_section.Unlock();

		
		while(SOCKET_ERROR == connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr)))
		{
			critical_section.Lock();
			if(exit_flag)
			{
				//shutdown(sockClient,SD_BOTH);
				//closesocket(sockClient);	
				critical_section.Unlock();
				ResetEvent(g_readyEvent_ConnectSocket);
				ReleaseMutex(socketMutex);
				return -1;
			}
			critical_section.Unlock();

			int errorCode = WSAGetLastError();
			if(errorCode == 10056 || errorCode == 10038)
			{
				closesocket(sockClient);

				while(INVALID_SOCKET == socket(AF_INET, SOCK_STREAM, IPPROTO_TCP))
				{
					//printf("COMM: Creating socket failed!\n");
					PostMessage(pInfo->hWnd,WM_DISPLAYSTATUS,NULL,STATUS_CREATE_SOCKET_FAIL);
					Sleep(1000);
				}
			}
			//printf("COMM: Connect socket failed!\n");
			PostMessage(pInfo->hWnd,WM_DISPLAYSTATUS,NULL,STATUS_CONNECT_SOCKET_FAIL);
			Sleep(2000);
		}
		PostMessage(pInfo->hWnd,WM_DISPLAYSTATUS,NULL,STATUS_CONNECT_SOCKET_SUCCESS);

		ResetEvent(g_readyEvent_ConnectSocket);
		ReleaseMutex(socketMutex);
	}

}

diffRptMsg_t headerBuf;
uint8 payloadBuf[MAX_PAYLOAD_BYTE_NUM];

//unsigned int __stdcall Thread_SendMsg(void *data)
unsigned int __stdcall Thread_SendMsg(LPVOID lpParam)
{
	//test for message comunicate
	ThreadProcInfo* pInfo =  (ThreadProcInfo*) lpParam;
	
	FILE *fpMsg = fopen(dataFileName,"rb");
	int readNum;
	int counter = 0;
	int nRet;


	while(1)
	{
		//for pause control
		critical_section.Lock();
		if(exit_flag)
		{
			critical_section.Unlock();
			fclose(fpMsg);
			//shutdown(sockClient,SD_SEND);
			shutdown(sockClient,SD_BOTH);
			closesocket(sockClient);	
			ResetEvent(g_readyEvent_Send); //reset g_readyEvent_Send to no signal
			return -1;
		}
		critical_section.Unlock();
		WaitForSingleObject(g_readyEvent_Send, INFINITE);
		
		uint8* recvBuffP = (uint8*)&headerBuf;
		int readLen;

		//step 1: read header length
		readLen = sizeof(headerBuf.msgHeader.headerLen);
		readNum = fread(&(headerBuf.msgHeader.headerLen), readLen, 1, fpMsg);
		if(readNum != 1)
		{
			fseek(fpMsg, 0, SEEK_SET);
			counter = 0;
			continue;
		}
		
		//read header
		recvBuffP += sizeof(headerBuf.msgHeader.headerLen);
		readLen = headerBuf.msgHeader.headerLen - sizeof(headerBuf.msgHeader.headerLen);
		readNum = fread(recvBuffP, readLen, 1, fpMsg);
		if(readNum != 1)
		{
			fseek(fpMsg, 0, SEEK_SET);
			counter = 0;
			continue;
		}

		//read payload
		readLen = headerBuf.msgHeader.payloadLen;
		if(readLen != 0)
		{
			readNum = fread(payloadBuf, readLen,1,fpMsg);
			if(readNum != 1)
			{
				fseek(fpMsg, 0, SEEK_SET);
				counter = 0;
				continue;
			}
		}
		
		if(headerBuf.msgHeader.payloadLen != 0)
		{
			//printf("message Count = %d\n",counter);
			//display message count
			PostMessage(pInfo->hWnd,WM_DISPLAYCOUNT,NULL,counter);
			counter++;

			//send header
			nRet = send(sockClient,(char*)&headerBuf,headerBuf.msgHeader.headerLen,0);
			if ((nRet == SOCKET_ERROR) || (nRet == 0))
			{
				trySetConnectSocket(true);
			}
			else
			{
				if((headerBuf.msgHeader.payloadLen > 0))
				{
					nRet = send(sockClient,(char*)payloadBuf,headerBuf.msgHeader.payloadLen,0);
					if ((nRet == SOCKET_ERROR) || (nRet == 0))
					{
						trySetConnectSocket(true);
					}
				}
			}
		
			Sleep(timeDelay);
		}
	}

	fclose(fpMsg);

}


unsigned int __stdcall Thread_RecvMsg(void *data)
{
	//int nNetTimeout = 30000;
	//setsockopt(sockClient,SOL_SOCKET,SO_RCVTIMEO,(char*)&nNetTimeout,sizeof(int));
	
	while(1)
	{

		int nRet = recv(sockClient,(char*)recvBuf,RECV_BUF_LEN,0);
		if((nRet == SOCKET_ERROR)|| (nRet == 0))
		{
			trySetConnectSocket(true);
			critical_section.Lock();
			if(exit_flag)
			{
				critical_section.Unlock();
				//shutdown(sockClient,SD_RECEIVE);
				shutdown(sockClient,SD_BOTH);
				closesocket(sockClient);
				return -1;
			}
			critical_section.Unlock();
			
			Sleep(2000);
		}
	}
}


void CInVehicle_EmulatorDlg::appInitEvents(void)
{
	g_readyEvent_ConnectSocket = CreateEvent(NULL,TRUE,FALSE,NULL);

	socketMutex = CreateMutex(NULL,FALSE,NULL);

	ReleaseMutex(socketMutex);

	//create events for recv thread
	g_readyEvent_Send = CreateEvent(NULL,TRUE,FALSE,NULL);
}

bool CInVehicle_EmulatorDlg::startSocket()
{
	
	// Initialize WinSock and check version  
    WORD wVersionRequested = MAKEWORD(2,0);  
    WSADATA wsaData;   
    
    int nRet = WSAStartup(wVersionRequested, &wsaData);  
    if (wsaData.wVersion != wVersionRequested)  
    {     
        //printf("COMM: The window socket version does not supoorted!");  
		AfxMessageBox(_T("The window socket version does not supoorted!"));
		return false;  
    } 
	
	//open the UDP socket client, connect to server side
    sockClient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sockClient == INVALID_SOCKET)  
    {  
        //printf("COMM: Creating socket failed!\n");
		AfxMessageBox(_T("Creating socket failed!"));
		return false;  
    }

	nRet = connect(sockClient,(struct sockaddr *)&serverAddr,sizeof(serverAddr));
	if(nRet == SOCKET_ERROR)
	{
		int errorCode = WSAGetLastError();
		//printf("COMM: Connect socket failed!\n");
		AfxMessageBox(_T("Connect socket fail,please check!"));
		return false;
	}
	
	SetDlgItemText(IDC_STATIC_STATUS_INDICATE, _T("socket connected"));

	return true;
}

int CInVehicle_EmulatorDlg::appStartImplement(ThreadProcInfo *ProcInfoParam)
{
	//start Socket
	startSocket();

	//HANDLE threadHandle[3];
	threadHandle[0] = (HANDLE) _beginthreadex(0,0,&Thread_RecvMsg,0,0,0);
	//test for message communitcate
	//threadHandle[1] = (HANDLE) _beginthreadex(0,0,&Thread_SendMsg,0,0,0);
	threadHandle[1] = (HANDLE) _beginthreadex(0,0,&Thread_SendMsg,(LPVOID)ProcInfoParam,0,0);
	//threadHandle[2] = (HANDLE) _beginthreadex(0,0,&Thread_ReconnectSocket,0,0,0);
	threadHandle[2] = (HANDLE) _beginthreadex(0,0,&Thread_ReconnectSocket,(LPVOID)ProcInfoParam,0,0);
	
	SetThreadPriority(threadHandle[0],THREAD_PRIORITY_NORMAL);
	SetThreadPriority(threadHandle[1],THREAD_PRIORITY_LOWEST);
	SetThreadPriority(threadHandle[2],THREAD_PRIORITY_NORMAL);

	//WaitForMultipleObjects(3, threadHandle, true, INFINITE);
	//CloseHandle(threadHandle[0]);
	//CloseHandle(threadHandle[1]);
	//CloseHandle(threadHandle[2]);

	return 0;
}
/*============================================================application code==========================================================*/


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


// CInVehicle_EmulatorDlg dialog




CInVehicle_EmulatorDlg::CInVehicle_EmulatorDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CInVehicle_EmulatorDlg::IDD, pParent)
	, m_strFilePath(_T(""))
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
	m_strFilePath = _T(".\\config\\messages.bin"); //set default path
	m_port = _T("6000"); //set default port
	m_timedelay = _T("3");
}

void CInVehicle_EmulatorDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
	DDX_Text(pDX, IDC_ROAD_DATA_FILE_EDIT, m_strFilePath);
	DDX_Control(pDX, IDC_SERVER_IPADDRESS, m_ServerIp);
	DDX_Text(pDX, IDC_PORT_EDIT, m_port);
	DDV_MaxChars(pDX, m_port, 65535);
	DDX_Control(pDX, IDC_START_BUTTON, m_startbutton);
	DDX_Control(pDX, IDC_STOP_BUTTON, m_stopbutton);
	DDX_Control(pDX, IDC_PAUSE_BUTTON, m_pausebutton);
	DDX_Text(pDX, IDC_TIMEDELAY_EDIT, m_timedelay);
	DDV_MaxChars(pDX, m_timedelay, 2000);
}

BEGIN_MESSAGE_MAP(CInVehicle_EmulatorDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	//test
	ON_MESSAGE(WM_DISPLAYCOUNT, &CInVehicle_EmulatorDlg::OnSendCounterDisplay)
	ON_MESSAGE(WM_DISPLAYSTATUS, &CInVehicle_EmulatorDlg::OnStatusDisplay)
	ON_BN_CLICKED(IDC_BROWSE_BUTTON, &CInVehicle_EmulatorDlg::OnClickedBrowseButton)
	ON_BN_CLICKED(IDC_START_BUTTON, &CInVehicle_EmulatorDlg::OnClickedStartButton)
	ON_BN_CLICKED(IDC_PAUSE_BUTTON, &CInVehicle_EmulatorDlg::OnClickedPauseButton)
	ON_BN_CLICKED(IDC_STOP_BUTTON, &CInVehicle_EmulatorDlg::OnClickedStopButton)
END_MESSAGE_MAP()


// CInVehicle_EmulatorDlg message handlers

BOOL CInVehicle_EmulatorDlg::OnInitDialog()
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
	
	//set default Server Ip Address
	CString  ServerIpdef =  _T("127.0.0.1");  
	m_ServerIp.SetWindowText(ServerIpdef);  
	m_stopbutton.EnableWindow(FALSE);
	
	//initial the app events
	appInitEvents();
	
	//for font set
	font.CreatePointFont(100,_T("Microsoft Sans Serif"),NULL);
	GetDlgItem(IDC_STATIC_SEND_COUNT)->SetFont(&font);
	GetDlgItem(IDC_STATIC_STATUS_INDICATE)->SetFont(&font);


	return TRUE;  // return TRUE  unless you set the focus to a control
}

void CInVehicle_EmulatorDlg::OnSysCommand(UINT nID, LPARAM lParam)
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

void CInVehicle_EmulatorDlg::OnPaint()
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
HCURSOR CInVehicle_EmulatorDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}



void CInVehicle_EmulatorDlg::OnClickedBrowseButton()
{
	// TODO: Add your control notification handler code here 

	TCHAR szFilter[] = _T("bin file(*.bin)|*.bin||");  

    CFileDialog fileDlg(TRUE, _T("bin"), NULL, 0, szFilter, this);
   
    if (IDOK == fileDlg.DoModal())   
    {     
        m_strFilePath = fileDlg.GetPathName();   
        SetDlgItemText(IDC_ROAD_DATA_FILE_EDIT, m_strFilePath);   
			
	}   
}


void CInVehicle_EmulatorDlg::OnClickedStartButton()
{
	// TODO: Add your control notification handler code here
	
	// make the control data to relevant variable
    UpdateData(TRUE); 
	
	//parameter config init	
	timeDelay = _ttoi(m_timedelay)*1000;
	//server address
	serverAddr.sin_family = AF_INET;
	DWORD sourceIP;
	//((CIPAddressCtrl*)GetDlgItem(IDC_SERVER_IPADDRESS))->GetAddress(sourceIP);
	m_ServerIp.GetAddress(sourceIP); 
	serverAddr.sin_addr.S_un.S_addr = htonl(sourceIP);
	//port
	int sPort;
	sPort = _ttoi(m_port);
	serverAddr.sin_port = htons(sPort);
	//The road data file path
	//m_strFilePath
	if(!PathFileExists(m_strFilePath))
	{
		AfxMessageBox(_T("The bin file is not exist!!!Please choose the correct file!!!"));
		return;
	}

	
	/*memset(dataFileName,0,sizeof(dataFileName));
	int i;
	int length = m_strFilePath.GetLength();  
	for(i=0;i<length;i++)  
    {  
        dataFileName[i] = m_strFilePath.GetAt(i);  
    }  
    dataFileName[i] = '\0';*/

	//test code for wide char
	
	memset(dataFileName,0,sizeof(dataFileName));
	int n = m_strFilePath.GetLength(); //get the wide char number
	int len = WideCharToMultiByte(CP_ACP,0,m_strFilePath,m_strFilePath.GetLength(),NULL,0,NULL,NULL);//get the wide char byte number
	if(len > FILE_PATH_LENGTH)
	{
		AfxMessageBox(_T("The length of file path is too long!!!"));
		return;
	}
	WideCharToMultiByte(CP_ACP,0,m_strFilePath,m_strFilePath.GetLength(),dataFileName,len,NULL,NULL); //wide char tranfer to multibyte 
	dataFileName[len+1] = '\0';

	//application start
	critical_section.Lock();
	exit_flag = FALSE;
	critical_section.Unlock();
	pause_flag =  FALSE;
	SetEvent(g_readyEvent_Send);
	
	//get the current window's handle
	procThreadInfo.hWnd = this->GetSafeHwnd();
	
	appStartImplement(&procThreadInfo);
	
	//disable the button
	 m_startbutton.EnableWindow(FALSE);  
	 m_stopbutton.EnableWindow(TRUE);
	 //MessageBox(_T("start send road data..."));
	//SetDlgItemText(IDC_STATIC_STATUS_INDICATE, _T("start send road data..."));
}


void CInVehicle_EmulatorDlg::OnClickedPauseButton()
{
	// TODO: Add your control notification handler code here
	//pause start
	if(m_startbutton.IsWindowEnabled() == FALSE)
	{
		if(!pause_flag)	
		{
			SetDlgItemText(IDC_PAUSE_BUTTON, _T("continue"));
			ResetEvent(g_readyEvent_Send); //reset g_readyEvent_Send to no signal,stop send
			pause_flag = TRUE;
		}
		else
		{
			//get current timeDelay			
			timeDelay = GetDlgItemInt(IDC_TIMEDELAY_EDIT)*1000;
			SetDlgItemText(IDC_PAUSE_BUTTON, _T("pause"));
			SetEvent(g_readyEvent_Send); //contine send
			pause_flag = FALSE;		
		}
	}
}


void CInVehicle_EmulatorDlg::OnClickedStopButton()
{
	// TODO: Add your control notification handler code here
	SetEvent(g_readyEvent_Send);
	
	critical_section.Lock();
	exit_flag = TRUE;
	critical_section.Unlock();

#if 1
	WaitForMultipleObjects(3, threadHandle, true, INFINITE);
	CloseHandle(threadHandle[0]);
	CloseHandle(threadHandle[1]);
	CloseHandle(threadHandle[2]);

	//close socket file
	//struct linger so_linger;
	//so_linger.l_onoff = 1;
	//so_linger.l_linger = 1000;
	//setsockopt(sockClient,SOL_SOCKET,SO_LINGER,(char*)&so_linger,sizeof(so_linger));
	//closesocket(sockClient);	
	shutdown(sockClient,SD_BOTH);
	closesocket(sockClient);
	
	critical_section.Lock();
	exit_flag = FALSE;
	critical_section.Unlock();
	SetDlgItemText(IDC_STATIC_SEND_COUNT, _T("message count = 0"));
	pause_flag =  FALSE;
	SetDlgItemText(IDC_PAUSE_BUTTON, _T("pause"));
	SetDlgItemText(IDC_STATIC_STATUS_INDICATE, _T(""));
	m_stopbutton.EnableWindow(FALSE);
	m_startbutton.EnableWindow(TRUE);
	
#endif
}


//for self defined message to display the current send counter
LRESULT CInVehicle_EmulatorDlg::OnSendCounterDisplay(WPARAM wParam, LPARAM lParam)
{
	CString temp;
	int n;
	
	n = (int)lParam;
	temp.Format(_T("message count = %d"), n);
	SetDlgItemText(IDC_STATIC_SEND_COUNT, temp);

	return 0;
}


LRESULT CInVehicle_EmulatorDlg::OnStatusDisplay(WPARAM wParam, LPARAM lParam)
{
	switch(lParam)
	{
		case	STATUS_CREATE_SOCKET_FAIL:
				SetDlgItemText(IDC_STATIC_STATUS_INDICATE, _T("socket disconnected"));
				break;
		
		case	STATUS_CONNECT_SOCKET_FAIL:
				SetDlgItemText(IDC_STATIC_STATUS_INDICATE, _T("socket disconnected"));
				break;
		case	STATUS_CONNECT_SOCKET_SUCCESS:
				SetDlgItemText(IDC_STATIC_STATUS_INDICATE, _T("socket connected"));
	}

	return 0;
}