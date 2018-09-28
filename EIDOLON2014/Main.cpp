#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <thread>

#if defined _WIN32
#include <windows.h>
#include <stdio.h>
#include <tchar.h>
#include <gl\GLU.h>

HANDLE testOutput;
#else
//TODO:impl linux
#endif

#include <string>

#include "Serial.h"
#include "ARS_Global.h"
#include "GLFW\glfw3.h"

/* Comportamentos */
#include "ARS_Main.h"
#include "ARSBehavior.h"
#include "ARSArduinoLAccel.h"
#include "ARSArduinoCom.h"
#include "ARSRobotController.h"
#include "AntTweakBar.h"
#include "ARS_TweekControl.h"

/** Variáveis controle **/
bool running = true;

/** Ajuda RENDERIZAÇÃO **/
GLdouble fovY            = 90; 
GLdouble zNear           = 0.00001;
GLdouble zFar	         = 10000.0;
enum ARSColor { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY, DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };

/** Variáveis openGL **/
GLFWwindow* window;

/** Gerente de comportamentos **/
ARS_Main* ars_main;

using namespace std;

void mouseClick(GLFWwindow * window, int button, int action, int mods){
	if(!TwEventMouseButtonGLFW(button, action)){}
}

void mousePos(GLFWwindow * window, double x, double y){
	if(!TwEventMousePosGLFW(x, y)){}
}

void keyFunc(GLFWwindow * window, int key, int scancode, int action, int mods){
	if(!TwEventKeyGLFW(key, action)){}
}

void charFunc(GLFWwindow * window, unsigned int c){
	if(!TwEventCharGLFW(c, GLFW_PRESS)){}
}

void window_size_callback(GLFWwindow* window, int width, int height)
{
	//Evento do tweek
	TwWindowSize(width, height);

	GLdouble aspect = (GLdouble)width/(GLdouble)height;

	/** Configura projeção **/	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	//glViewport(0,0,width,height);
	GLdouble fW, fH;
	fH = tan( (fovY / 2) / 180 * pi ) * zNear;
	fH = tan( fovY / 360 * pi ) * zNear;
	fW = fH * aspect;
	glFrustum( -fW, fW, -fH, fH, zNear, zFar );

	/** Configura translação inicial **/
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

/** Inicialização opengl **/
int ARS_InitializeGLFW(){

	printf("Inicializando GLFW - ");
	//Sleep(500);

	if(!glfwInit()){
		ARS_PrintFail();
		return ARS_FAILED_TO_INIT_OGL;
	}

	ARS_PrintOK();

	//glfwWindowHint(GLFW_RESIZABLE, false);

	window = glfwCreateWindow(
		ARS_DEFAULT_WINDOW_WIDTH,
		ARS_DEFAULT_WINDOW_HEIGHT,
		"EIDOLON",  NULL, NULL);

	if(!window){
		glfwTerminate();
		return ARS_FAILED_TO_OPEN_WINDOW;
	}

	glfwMakeContextCurrent(window);
	
	//Inicializa tweek
	TwInit(TW_OPENGL, NULL);
	TwWindowSize(ARS_DEFAULT_WINDOW_WIDTH, ARS_DEFAULT_WINDOW_HEIGHT);

	/** Configuração de callbacks **/
	glfwSetWindowSizeCallback(window, window_size_callback);
	glfwSetMouseButtonCallback(window, mouseClick);
	glfwSetCursorPosCallback(window, mousePos);
	glfwSetKeyCallback(window, keyFunc);
	glfwSetCharCallback(window, charFunc);
	
 return ARS_OK;
}

int ARS_ConfigureOGL(){

	printf("Configura OpenGL - ");
	//Sleep(500);

	GLdouble aspect = (GLdouble)ARS_DEFAULT_WINDOW_WIDTH/(GLdouble)ARS_DEFAULT_WINDOW_HEIGHT;

	glClearColor(0.8f,0.8f,0.8f,1.0f);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glEnable(GL_DEPTH_TEST);

	/** Configura projeção **/	
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	GLdouble fW, fH;
	fH = tan( (fovY / 2) / 180 * pi ) * zNear;
	fH = tan( fovY / 360 * pi ) * zNear;
	fW = fH * aspect;
	glFrustum( -fW, fW, -fH, fH, zNear, zFar );

	/** Configura translação inicial **/
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	ARS_PrintOK();

	return ARS_OK;
}

int initializeComponents(){


	/** Create windows JOB manager **/
	printf("Criando JOB para gerenciar tarefas do cotrolador - ");
	//Sleep(500);
	thisJob = CreateJobObject(NULL, NULL);

	if(thisJob == NULL){
		ARS_PrintFail();
		getchar();
		exit(-1);
	}

	ARS_PrintOK();

	printf("Configurando programa - ");
//	Sleep(500);
	serialComConsole = ARS_CreateChildConsole(ARS_LW);
	timeCheckTerminal = ARS_CreateChildConsole(ARS_LB);

	

	if(serialComConsole >= 0 && timeCheckTerminal >= 0)
		ARS_PrintOK();
	else
		ARS_PrintFail();

	/** Configura job atual **/
	JOBOBJECT_EXTENDED_LIMIT_INFORMATION  jobinfo;
	jobinfo.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
	SetInformationJobObject(
		thisJob,
		JobObjectExtendedLimitInformation,
		&jobinfo,
		sizeof(JOBOBJECT_EXTENDED_LIMIT_INFORMATION));

	printf("Criando comportamentos - ");
	//Sleep(500);

	ars_main = new ARS_Main();

	/** Inserir todos os comportamentos aqui **/
	ARSArduinoCom	  * ardCom		= new ARSArduinoCom		("COM3");
	ARSArduinoLAccel  * ardAccel	= new ARSArduinoLAccel	(ardCom);
	ARSRobotController* robCtrl		= new ARSRobotController("COM6", ardAccel);

	//Configura tweek 
	robot = robCtrl;
	createAllBars();

	ars_main->insertBehavior(ardCom  );
	ars_main->insertBehavior(ardAccel);
	ars_main->insertBehavior(robCtrl );

	ARS_PrintOK();

	//Sleep(500);
	printf("------ Inicializando comportamentos ------\n");
	//Sleep(500);

	/** Inicializa totos os comportamentos **/
	ars_main->initialize();

	return ARS_OK;
}
	// Para desenhar partes

double meanA = 0.0;
int    itA = 0;

int main(int argc, char* argv[], char* envp[]){
	/** Conf initial console **/
	SetColor(WHITE);

	//ARS_MainCommunicationTest();
	ARS_InitializeGLFW	();
	ARS_ConfigureOGL	();
	initializeComponents(); 

	oldtime = glfwGetTime();

	while(running){
		currenttime = glfwGetTime();
		deltatime   = currenttime - oldtime;
		oldtime     = currenttime;

		double timeCheckStart = glfwGetTime();

		// Atualiza modulos aqui
		ars_main->update();

		// Renderizar 
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glLoadIdentity();
		gluLookAt(0,0,10,0,0,0,0,1,0);
		//glTranslatef(0,0,-20);

		ars_main->draw();
		TwDraw();

		// Troca buffers para rasterização 
		glfwSwapBuffers(window);

		// Tratar eventos da janela 
		glfwPollEvents();

		//printf("COMPUTADOR\n");
		double timeCheckEnd = glfwGetTime();
		double execTime     = timeCheckEnd - timeCheckStart;

		meanA += execTime;
		itA ++;

		string toSend       = "-----loop principal-";
		toSend.append(std::to_string(meanA / (double)itA));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());
		//printf("TESTE\n");
		//tempoTotal.push_back(execTime);
	}

	// Wait until child process exits.

	glfwTerminate();

	return ARS_OK;
}
