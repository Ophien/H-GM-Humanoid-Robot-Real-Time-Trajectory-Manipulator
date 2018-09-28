#include "ARSArduinoCom.h"

ARSArduinoCom::ARSArduinoCom(char* port)
{
	this->port = port;
}

ARSArduinoCom::~ARSArduinoCom(void)
{
}

void ARSArduinoCom::initialize(){
	arduino = new Serial(port);

	if(arduino == NULL)
		ARS_TreatError(ARS_FAILED_TO_CONNECT_ARDUINO);

	readResult = -1;
}

string ARSArduinoCom::getLastReaded(){
	//Valor padrão para retorno
	string toReturn = "";

	//Copiar resultado para string
	if(readResult != -1)
		toReturn.append(receivedData);

	//Limpar buffer de leitura
	for(int i = 0; i < ARS_READ_BUFFER_SIZE; i++)
		receivedData[i]= '\0';

	//Configura estado do comportamento
	readResult = -1;

	return toReturn;
}

void ARSArduinoCom::preUpdate(){}

void ARSArduinoCom::update(){

	readResult = arduino->ReadData(receivedData, ARS_READ_BUFFER_SIZE);
}

void ARSArduinoCom::posUpdate(){

}

void ARSArduinoCom::draw(){
}