#pragma once

#include "ARS_Global.h"
#include "ARSBehavior.h"
#include "Serial.h"
#include <string>

using namespace std;

#define ARS_READ_BUFFER_SIZE 1024

class ARSArduinoCom : public ARSBehavior
{
public:
	ARSArduinoCom(char* port);
	~ARSArduinoCom(void);

	void   initialize();
	void   update();
	void   preUpdate();
	void   posUpdate();
	void draw();

	string getLastReaded();
	
private:
	Serial* arduino;
	char  * port; 
	
	//Controla de leitura
	char receivedData[ARS_READ_BUFFER_SIZE];
	int  readResult;
};

