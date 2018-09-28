#include "ARS_Global.h"
#include <string>

HANDLE thisJob;
std::vector<HANDLE> terminalOutputs;
int    consoleCount     = 0;
int    serialComConsole;// = ARS_CreateChildConsole(ARS_LW);
int timeCheckTerminal;
double oldtime     = 0.0;
double currenttime = 0.0;
double deltatime   = 0.0;
bool sendToConsole = false;
//testes

std::vector<double> tempoTotal;
std::vector<double> tempoComportRobo;
std::vector<double> tempoComportAccel;

enum ARSColor { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY, DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };

Serial* ARS_ConnectDevice(char* port){
	Serial* arduinoPort;

	std::string send = ("Conectando na porta ");
	send.append(port);
	send.append("...\n");
	ARS_WriteToPipe(serialComConsole,send.c_str());

	arduinoPort = new Serial(port);

	if(arduinoPort->IsConnected()){
		ARS_WriteToPipe(serialComConsole, "Conectado com sucesso...\n");
	}else{
		ARS_WriteToPipe(serialComConsole, "Falha na conexão do dispositivo... Precione qualquer tecla para continuar\n");
		getchar();
		return NULL;
	}

	return arduinoPort;
}

void SetColor(ARSColor c){
	HANDLE hCon = NULL;
    if(hCon == NULL)
        hCon = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hCon, c);
}

void ARS_PrintOK(){
	SetColor(GREEN);
	printf("OK\n");
	SetColor(WHITE);
}

void ARS_PrintFail(){
	SetColor(RED);
	printf("FAIL\n");
	SetColor(WHITE);
}

void ARS_PrintERRO(char* erro){
	SetColor(RED);
	printf(erro);
	SetColor(WHITE);
}

void ARS_TreatError(int code){
	switch(code){
	case ARS_FAILED_TO_CONNECT_ARDUINO:
		break;
	case ARS_FAILED_TO_INIT_OGL:
		break;
	case ARS_FAILED_TO_OPEN_WINDOW:
		break;
	case ARS_DEFAULT_WINDOW_WIDTH:
		break;
	case ARS_DEFAULT_WINDOW_HEIGHT:
		break;
	}

	exit(code);
}

//Cria processo filho de forma que o mesmo tenha como input, o que é gravado no pipe
int ARS_CreateChildConsole(char* color){
	HANDLE outputPinParent = NULL; // Para input do pai, grava no pai
	HANDLE outputPin       = NULL; // Para input do filho, le do filho
	SECURITY_ATTRIBUTES saAttr; 

	saAttr.nLength = sizeof(SECURITY_ATTRIBUTES); 
	saAttr.bInheritHandle = TRUE; 
	saAttr.lpSecurityDescriptor = NULL; 

	if (! CreatePipe(&outputPin, &outputPinParent, &saAttr, 0)) 
		ARS_ErrorExit(TEXT("Stdin CreatePipe")); 

	if ( ! SetHandleInformation(outputPinParent, HANDLE_FLAG_INHERIT, 0) )
		ARS_ErrorExit(TEXT("Stdin SetHandleInformation")); 

	PROCESS_INFORMATION piProcInfo; 
	STARTUPINFO siStartInfo;
	BOOL bSuccess = FALSE; 

	ZeroMemory( &piProcInfo , sizeof(PROCESS_INFORMATION) );
	ZeroMemory( &siStartInfo, sizeof(STARTUPINFO) );

	siStartInfo.cb = sizeof(STARTUPINFO); 
	// siStartInfo.hStdError  = g_hChildStd_OUT_Wr;
	// siStartInfo.hStdOutput = g_hChildStd_OUT_Wr;
	siStartInfo.hStdInput    = outputPin; //Configura input do novo processo para saida do pipe
	siStartInfo.dwFlags     |= STARTF_USESTDHANDLES;

	std::string path = "F:\\robotica\\td\\refs\\PROGRAM\\EIDOLON 2014\\EIDOLON2014\\Debug\\ARSSimpleConsole.exe";
	std::string comd = "ARSSimpleConsole.exe ";
	comd.append(color);
	char buf[1024];// = new char(comd.length());
	strcpy_s(buf, 1024, comd.c_str());

	bSuccess = CreateProcess(
		path.c_str(), 
		buf,         // command line 
		NULL,          // process security attributes 
		NULL,          // primary thread security attributes 
		TRUE,          // handles are inherited 
		CREATE_NEW_CONSOLE, // creation flags 
		NULL,          // use parent's environment 
		NULL,          // use parent's current directory 
		&siStartInfo,  // STARTUPINFO pointer 
		&piProcInfo);  // receives PROCESS_INFORMATION 

	if ( ! bSuccess ) 
		ARS_ErrorExit(TEXT("CreateProcess"));
	else 
	{
		// Close handles to the child process and its primary thread.
		// Some applications might keep these handles to monitor the status
		// of the child process, for example. 

		//Coloca processo criado no job deste programa
		AssignProcessToJobObject(thisJob, piProcInfo.hProcess);

		CloseHandle(piProcInfo.hProcess);
		CloseHandle(piProcInfo.hThread);
	}

	//Garante referencia do HANDLER
	terminalOutputs.push_back(outputPinParent);

	// Cria identificador para novo terminal
	int consoleId = consoleCount;
	consoleCount++;

	return consoleId;
}

void ARS_WriteToPipe(unsigned int console, const char* message) 

	// Read from a file and write its contents to the pipe for the child's STDIN.
	// Stop when there is no more data. 
{ 

	if(!sendToConsole)
		return;

	if(console >= terminalOutputs.size()){
		ARS_TreatError(ARS_CONSOLE_INVALIDO);
	}

	DWORD dwWritten;
	BOOL bSuccess = WriteFile(terminalOutputs[console], message, 
		strlen(message), &dwWritten, NULL);

	if(!bSuccess){
		ARS_PrintERRO("Nao foi possivel enviar menssagem para processo especificado\n");
	} 
}

	void ARS_ErrorExit(PTSTR lpszFunction) 

		// Format a readable error message, display a message box, 
		// and exit from the application.
	{ 
		LPVOID lpMsgBuf;
		LPVOID lpDisplayBuf;
		DWORD dw = GetLastError(); 

		FormatMessage(
			FORMAT_MESSAGE_ALLOCATE_BUFFER | 
			FORMAT_MESSAGE_FROM_SYSTEM |
			FORMAT_MESSAGE_IGNORE_INSERTS,
			NULL,
			dw,
			MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
			(LPTSTR) &lpMsgBuf,
			0, NULL );

		lpDisplayBuf = (LPVOID)LocalAlloc(LMEM_ZEROINIT, 
			(lstrlen((LPCTSTR)lpMsgBuf)+lstrlen((LPCTSTR)lpszFunction)+40)*sizeof(TCHAR)); 
		StringCchPrintf((LPTSTR)lpDisplayBuf, 
			LocalSize(lpDisplayBuf) / sizeof(TCHAR),
			TEXT("%s failed with error %d: %s"), 
			lpszFunction, dw, lpMsgBuf); 
		MessageBox(NULL, (LPCTSTR)lpDisplayBuf, TEXT("Error"), MB_OK); 

		LocalFree(lpMsgBuf);
		LocalFree(lpDisplayBuf);
		ExitProcess(1);
	}