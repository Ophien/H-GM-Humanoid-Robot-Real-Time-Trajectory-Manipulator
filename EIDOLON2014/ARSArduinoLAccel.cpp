#include "ARSArduinoLAccel.h"
#include "ARS_Global.h"
#include "GLFW\glfw3.h"

ARSArduinoLAccel::ARSArduinoLAccel(ARSArduinoCom* comBehavior)
{
	this->comBehavior = comBehavior;
	curState          = INICIAL;
	x				  = "";
	y				  = "";
	z				  = "";
}

ARSArduinoLAccel::~ARSArduinoLAccel(void)
{
}

void ARSArduinoLAccel::initialize(){
	accelConsole        = ARS_CreateChildConsole(ARS_LG);
	pitchYawRollConsole = ARS_CreateChildConsole(ARS_LY);
}

void ARSArduinoLAccel::preUpdate(){}
void ARSArduinoLAccel::posUpdate(){}

double meanC = 0.0;
int    itC = 0;

void ARSArduinoLAccel::update(){
		double timeCheckStart = glfwGetTime();

	string currentData = comBehavior->getLastReaded();

	//Parser para pegar dado do sensor, separa em x y e z
	for(unsigned int i = 0; i < currentData.length(); i++){
		switch(curState){
		case INICIAL:
			if(currentData[i] == '<')
				curState = GET_X;
			break;
		case GET_X:
			if(currentData[i] != '\t')
				x += currentData[i];
			else
				curState = GET_Y;
			break;
		case GET_Y:
			if(currentData[i] != '\t')
				y += currentData[i];
			else
				curState = GET_Z;
			break;
		case GET_Z:
			if(currentData[i] != '>')
				z += currentData[i];
			else
			{
				// LEU UM ELEMENTO COMPLETO, ARMAZENAR NA LISTA EM UM VECTOR3
				Vector3f accel((float)atoi(x.c_str()), (float)atoi(y.c_str()), (float)atoi(z.c_str()));
				accel.normalize();

				accel.x += ACCEL_X_OFFSET;
				accel.y += ACCEL_Y_OFFSET;
				accel.z += ACCEL_Z_OFFSET;

				availableAccel.push_back(accel);

				Vector3f back = availableAccel.back();

				ARS_WriteToPipe(accelConsole, back.toString().c_str());

				x.clear();
				y.clear();
				z.clear();

				curState = STAT_GIRO;
			}
			break;
		case STAT_GIRO:
			if(currentData[i] == '@')
				curState = GET_YA;
			break;
		case GET_YA:
			if(currentData[i] != '\t')
				x += currentData[i];
			else
				curState = GET_P;
			break;
		case GET_P:
			if(currentData[i] != '\t')
				y += currentData[i];
			else
				curState = GET_R;
			break;
		case GET_R:
			if(currentData[i] != '@')
				z += currentData[i];
			else
			{
				// LEU UM ELEMENTO COMPLETO, ARMAZENAR NA LISTA EM UM VECTOR3
				availableYawPitchRoll.push_back(Vector3f((float)atoi(x.c_str()), (float)atoi(y.c_str()), (float)atoi(z.c_str())));

				Vector3f back = availableYawPitchRoll.back();

				ARS_WriteToPipe(pitchYawRollConsole, back.toStringYPR().c_str());

				x.clear();
				y.clear();
				z.clear();

				curState = INICIAL;
			}
		}
	}



			double timeCheckEnd = glfwGetTime();
		double execTime     = timeCheckEnd - timeCheckStart;

					meanC += execTime;
		itC ++;

		string toSend       = "------------Arduino-";
		toSend.append(std::to_string(meanC/(double)itC));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());
}

void ARSArduinoLAccel::draw(){
}
