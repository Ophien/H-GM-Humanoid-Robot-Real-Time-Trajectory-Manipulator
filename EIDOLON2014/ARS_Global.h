#pragma once

#define ARS_OK 0
#define ARS_FAILED_TO_CONNECT_ARDUINO -1000
#define ARS_FAILED_TO_CONNECT_MICROCONTROLER -1001
#define ARS_FAILED_TO_INIT_OGL    -999
#define ARS_FAILED_TO_OPEN_WINDOW -998
#define ARS_CONSOLE_INVALIDO -997

#define ARS_INVALID_EFFECTOR_ID -200

#define ARS_DEFAULT_WINDOW_WIDTH  1920
#define ARS_DEFAULT_WINDOW_HEIGHT 1080

#define ARS_TOTAL_CONSOLES              8
#define ARS_ACCELEROMETER_CONSOLE		0
#define ARS_ROBOT_LEFTFOOT_CONSOLE		1
#define ARS_ROBOT_RIGHTFOOT_CONSOLE		2
#define ARS_PSO_CONSOLE                 3
#define ARS_ARDUINO_COM                 4
#define ARS_MICROCONTROLER_COM          5
#define ARS_OGL_CONSOLE					6
#define ARS_ERROR_CONSOLE				7

#define ARS_BLACK	 "0"
#define ARS_BLUE	 "1"
#define ARS_GREEN	 "2"
#define ARS_AQUA	 "3"
#define ARS_RED		 "4"
#define ARS_PURPLE	 "5"
#define ARS_YELLOW	 "6"
#define ARS_WHITE	 "7"
#define ARS_GRAY	 "8"
#define ARS_LB		 "9"
#define ARS_LG		 "A"
#define ARS_LA		 "B"
#define ARS_LR		 "C"
#define ARS_LP		 "D"
#define ARS_LY		 "E"
#define ARS_LW		 "F"

#include "Serial.h"

#include <windows.h> 
#include <tchar.h>
#include <stdio.h> 
#include <strsafe.h>
#include <vector>

#define BUFSIZE 4096 

/** Gerenciar terminais **/
extern HANDLE thisJob;
extern std::vector<HANDLE> terminalOutputs;
extern int    consoleCount;
extern enum ARSColor;
extern int    serialComConsole;
extern double oldtime;
extern double currenttime;
extern double deltatime;
extern int timeCheckTerminal;
extern bool sendToConsole;

/** TESTES **/
extern std::vector<double> tempoTotal;
extern std::vector<double> tempoComportRobo;
extern std::vector<double> tempoComportAccel;

int  ARS_CreateChildConsole(char* color = ""); 
void ARS_WriteToPipe(unsigned int output, const char* message); 
void ARS_ErrorExit(PTSTR); 


void SetColor(ARSColor c);
void ARS_PrintERRO(char* erro);
void ARS_PrintOK();
void ARS_PrintFail();

/** Comunicação e conexão com dispositivo **/
Serial* ARS_ConnectDevice(char* port);
void ARS_TreatError(int code);

