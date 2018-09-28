#include "ARSRobotController.h"
#include "BSplineGLDrawer.h"
#include "ARS_MathHelper.h"
#include "GLFW\glfw3.h"
#include <gl\GLU.h>
#include <vector>

ARSRobotController::ARSRobotController(char* comPort,  ARSArduinoLAccel* giroscopeAccelModule)
{
	port = comPort;
	this->giroscopeAccel = giroscopeAccelModule;
}

ARSRobotController::~ARSRobotController(void)
{
}

void ARSRobotController::initialize(){
	//rightFootPositionConsole = ARS_CreateChildConsole(ARS_LW);
	robotCom = new Serial(port);

	if(robotCom == NULL)
		ARS_TreatError(ARS_FAILED_TO_CONNECT_MICROCONTROLER);

	// Regulate com and robot
	defaultStandInstance(robotCom);

	defaultVirtualEffectorPosition(NULL);

	// Para desenhar partes
	sphere = gluNewQuadric();
	gluQuadricDrawStyle(sphere,GLU_LINE);

	baseSagital  = 4.0f;
	sagitalTresh = 20.0f;
	baseFrontal  = 0.0f;
	frontalTresh = 20.0f;

	b_giroscopeAssertingRateL = 3.0f;
	b_giroscopeAssertingRateR = 3.0f;

	//enable = false;
	footRotationRollThreshold			= 1.0f;
	footRotationPitchThreshold			= 1.0f;
	footRotationPitchOffset			    = 0.0f;
	footRotationRollOffset			    = 0.0f;
	footRotationRollEquilibriumIterator = 0.0f;
	sagitalFootEquilibrium			    = -12.00000238f;

	timeFactor						    = 0.4f;
	frontalDisplacement				    = 1.0f;
	sagitalStepLenght				    = 3.0f;
	sagitalStepHeight				    = 4.0f;

	displacementY					    = .0f;

	minimumStepHeight = 1.0f;
	minimumStepLenght = 1.0f;
	maximumStepHeight = 10.0f;
	maximumStepLenght = 10.0f;

	b_sagitalStepLenghtL	  = 5.0f;
	b_sagitalStepLenghtR	  = 5.0f;
	b_sagitalStepHeightL	  = 5.0f;
	b_sagitalStepHeightR	  = 5.0f;

	b_frontalDisplacementL	  = 5.5f;
	b_frontalDisplacementR    = 5.5f;

    b_frontalLfootrotratio    = 1.0f;
	b_frontalRfootrotratio    = 1.0f;

	b_stepLenghtDisplaceL     = 1.0f;
	b_stepLenghtDisplaceR     = 1.0f;

	b_directionL			  = 1.0f;
	b_directionR			  = 1.0f;

	b_parametricL			  = 1.0f;
	b_parametricR			  = 0.0f;

	b_parametricRatioL		  = 0.6f;//velocidade
	//b_parametricRatioR		  = 8.6f;//velocidade

	b_stepLenghtHipDisplaceLR = 1.0f;
	b_stepLenghtHipDisplaceLF = 0.0f;
	b_stepLenghtHipDisplaceRR = 1.0f;
	b_stepLenghtHipDisplaceRF = 0.0f;

	b_hipTrajectoryHeightLR	  = 0.0f;
	b_hipTrajectoryHeightLF	  = 0.0f;
	b_hipTrajectoryHeightRR   = 0.0f;
	b_hipTrajectoryHeightRF	  = 0.0f;

	leftFootTrajectory.controlPoints.push_back(Vector3f(-2,1,0));
	leftFootTrajectory.controlPoints.push_back(Vector3f(-1,4,0));
	leftFootTrajectory.controlPoints.push_back(Vector3f( 1,4,0));
	leftFootTrajectory.controlPoints.push_back(Vector3f( 2,1,0));

	rightFootTrajectory.controlPoints.push_back(Vector3f(-2,1,0));
	rightFootTrajectory.controlPoints.push_back(Vector3f(-1,4,0));
	rightFootTrajectory.controlPoints.push_back(Vector3f( 1,4,0));
	rightFootTrajectory.controlPoints.push_back(Vector3f( 2,1,0));

	leftFootHipTrajectory.controlPoints.push_back(Vector3f(-2,1,0));
	leftFootHipTrajectory.controlPoints.push_back(Vector3f(-1,4,0));
	leftFootHipTrajectory.controlPoints.push_back(Vector3f( 1,4,0));
	leftFootHipTrajectory.controlPoints.push_back(Vector3f( 2,1,0));

	rightFootHipTrajectory.controlPoints.push_back(Vector3f(-2,0,0));
	rightFootHipTrajectory.controlPoints.push_back(Vector3f(-1,0,0));
	rightFootHipTrajectory.controlPoints.push_back(Vector3f( 1,0,0));
	rightFootHipTrajectory.controlPoints.push_back(Vector3f( 2,0,0));


	//controle simples
	assertByGiroscope = false;
	useDirectTransition = false;
	sendDataSagital = false;
	sendDataFrontal = false;
	useEquilibrium  = false;
	usePolinom      = false;
	useElipsoid     = false;
	invertFrontal   = false;
}

void ARSRobotController::draw(){
	float drawYDisplace = 5;
	float drawXDisplace = 0;
	float drawZDisplace = 5;

	// Desenha passo sagital
	for(int i = 0; i < 3; i++){
		glPushMatrix();
		glTranslatef(sagitalRightLeg[i].x - drawXDisplace, sagitalRightLeg[i].y -drawYDisplace, sagitalRightLeg[i].z);
		glColor3f(0,0,0);
		gluSphere(sphere, 1, 10, 10);
		glPopMatrix();

		glPushMatrix();
		glTranslatef(sagitalLeftLeg[i].x - drawXDisplace, sagitalLeftLeg[i].y -drawYDisplace, sagitalLeftLeg[i].z - drawZDisplace);
		glColor3f(0,0,0);
		gluSphere(sphere, 1, 10, 10);
		glPopMatrix();
	}

	// Desenha passo frontal
	/*for(int i = 0; i < 2; i++){
	glPushMatrix();
	glTranslatef(frontalRightLeg[i].x + drawXDisplace, frontalRightLeg[i].y -drawYDisplace, frontalRightLeg[i].z);
	glColor3f(0,1,0);
	gluSphere(sphere, 1, 10, 10);
	glPopMatrix();
	}*/

	/** Linha representativa chao **/
	/*
	1) top left
	2) bottom left
	3) top right
	4) bottom right
	*/
	glColor3f(0,0,0);
	glPushMatrix();
	glBegin(GL_LINES);
	glVertex3f(-100,displacementY -drawYDisplace,0);
	glVertex3f( 100,displacementY -drawYDisplace,0);
	glEnd();
	glPopMatrix();

	glColor3f(0,0,0);
	glPushMatrix();
	glBegin(GL_LINES);
	glVertex3f(-100,displacementY -drawYDisplace,-drawZDisplace);
	glVertex3f( 100,displacementY -drawYDisplace,-drawZDisplace);
	glEnd();
	glPopMatrix();

	/** Ligamento entre partes **/
	// Desenha passo sagital
	for(int i = 0; i < 2; i++){
		glColor3f(0,0,0);
		glPushMatrix();
		glBegin(GL_LINES);
		glVertex3f(sagitalRightLeg[i].x - drawXDisplace,sagitalRightLeg[i].y -drawYDisplace,0);
		glVertex3f(sagitalRightLeg[i+1].x - drawXDisplace,sagitalRightLeg[i+1].y -drawYDisplace,0);
		glEnd();
		glPopMatrix();

		glColor3f(0,0,0);
		glPushMatrix();
		glBegin(GL_LINES);
		glVertex3f(sagitalLeftLeg[i].x - drawXDisplace,sagitalLeftLeg[i].y -drawYDisplace,-drawZDisplace);
		glVertex3f(sagitalLeftLeg[i+1].x - drawXDisplace,sagitalLeftLeg[i+1].y -drawYDisplace,-drawZDisplace);
		glEnd();
		glPopMatrix();
	}

	// Desenha passo frontal
	for(int i = 0; i < 1; i++){
		glColor3f(0,0,1);
		glPushMatrix();
		glBegin(GL_LINES);
		glVertex3f(frontalRightLeg[i].x   + drawXDisplace,frontalRightLeg[i].y -drawYDisplace,0);
		glVertex3f(frontalRightLeg[i+1].x + drawXDisplace,frontalRightLeg[i+1].y -drawYDisplace,0);
		glEnd();
		glPopMatrix();
	}

	// Desenha passo frontal
	for(int i = 0; i < 1; i++){
		glColor3f(1,0,0);
		glPushMatrix();
		glBegin(GL_LINES);
		glVertex3f(frontalLeftLeg[i].x   + drawXDisplace,frontalLeftLeg[i].y -drawYDisplace,0);
		glVertex3f(frontalLeftLeg[i+1].x + drawXDisplace,frontalLeftLeg[i+1].y -drawYDisplace,0);
		glEnd();
		glPopMatrix();
	}

	/** desenha curvas e cv **/

	glColor3f(0.5,0.5,0.5);
	glPushMatrix();
	glBegin(GL_LINES);
	glVertex3f(-100,displacementY -drawYDisplace,0);
	glVertex3f( 100,displacementY -drawYDisplace,0);
	glEnd();
	glPopMatrix();

	glColor3f(0.5,0.5,0.5);
	glPushMatrix();
	glBegin(GL_LINES);
	glVertex3f(0,100,0);
	glVertex3f( 0,-100,0);
	glEnd();
	glPopMatrix();

	if(usePolinom){
		DrawSpline(&rightFootTrajectory, Vector3f(-drawXDisplace,-drawYDisplace,0.0f), Vector3f(0,0,0), Vector3f(0,0,0), Vector3f(0.0f,0.0f, 0.0f));
		DrawSpline(&rightFootHipTrajectory, Vector3f(-drawXDisplace,-drawYDisplace,0.0f), Vector3f(0.0f,0.0f,0.0f), Vector3f(0,0,0), Vector3f(0.0f,0.0f, 0.0f));
		DrawSpline(&leftFootTrajectory, Vector3f(-drawXDisplace,-drawYDisplace,-drawZDisplace), Vector3f(0,0,0), Vector3f(0,0,0), Vector3f(0.0f,0.0f, 0.0f));
		DrawSpline(&leftFootHipTrajectory, Vector3f(-drawXDisplace,-drawYDisplace,-drawZDisplace), Vector3f(0.0f,0.0f,0.0f), Vector3f(0,0,0), Vector3f(0.0f,0.0f, 0.0f));
	}
}

void ARSRobotController::preUpdate(){
}

/*
@ -> UPPER THIGH frontalLeg[0]
|
|
@ -> FOOT        frontalLeg[1]
*/
void ARSRobotController::calculateAngleFromFrontalPosition(){

	/*************************DIREITA***************************/
	/** Para angulo da coxa baixa, cintura para o eixo Y**/
	Vector3f TK     = frontalRightLeg[1] - frontalRightLeg[0];
	Vector3f Y_AXIS = Vector3f(0,1,0);
	int upperThighAngle = 0;
	upperThighAngle = ARSAngleBetweenVectors(TK,Y_AXIS);// Lower thigh

	float finalUpperThighAngle = (360.0f - upperThighAngle*2.0f)/2.0f;

	if(invertFrontal)
		finalUpperThighAngle *= -1.0f;

	if(TK.x < 0)
		finalUpperThighAngle = (finalUpperThighAngle) * -1;

	servoCurrentAngle[ARS_RIGHT_UPPER_THIGH] = (int)finalUpperThighAngle;
	servoCurrentAngle[ARS_RIGHT_FOOT       ] =(int)(
		finalUpperThighAngle*b_frontalRfootrotratio- 
		footRotationPitchOffset*footRotationPitchThreshold);

	if(invertFrontal)
		servoCurrentAngle[ARS_RIGHT_FOOT       ] *= -1.0f;

	if(assertByGiroscope){
		servoCurrentAngle[ARS_RIGHT_FOOT       ] = +finalUpperThighAngle*b_frontalRfootrotratio+footRotationPitchOffset;// * b_giroscopeAssertingRateR;
	}
	/*************************ESQUERDA***************************/
	/** Para angulo da coxa baixa, cintura para o eixo Y**/
	TK     = frontalLeftLeg[1] - frontalLeftLeg[0];
	Y_AXIS = Vector3f(0,1,0);
	upperThighAngle = 0;
	upperThighAngle = ARSAngleBetweenVectors(TK,Y_AXIS);// Lower thigh

	finalUpperThighAngle = (360.0f - upperThighAngle*2.0f)/2.0f;

	if(TK.x < 0)
		finalUpperThighAngle = (finalUpperThighAngle) * -1;

	if(invertFrontal)
		finalUpperThighAngle *= -1.0f;

	servoCurrentAngle[ARS_LEFT_UPPER_THIGH] = (int)-finalUpperThighAngle;
	servoCurrentAngle[ARS_LEFT_FOOT       ] = (int)(
		-finalUpperThighAngle *b_frontalLfootrotratio+ 
		footRotationPitchOffset*footRotationPitchThreshold);

	if(invertFrontal)
		servoCurrentAngle[ARS_LEFT_FOOT       ] *= -1.0f;

	if(assertByGiroscope){
		servoCurrentAngle[ARS_LEFT_FOOT       ] = -finalUpperThighAngle*b_frontalLfootrotratio-footRotationPitchOffset;// * b_giroscopeAssertingRateL;
	}
}

void ARSRobotController::mirrorPolinomFunc(){
	b_sagitalStepLenghtR	  = b_sagitalStepLenghtL;
	b_sagitalStepHeightR	  = b_sagitalStepHeightL;
	b_frontalDisplacementR    = b_frontalDisplacementL;
	b_stepLenghtDisplaceR     = b_stepLenghtDisplaceL;
	b_stepLenghtHipDisplaceRR = b_stepLenghtHipDisplaceLR;
	b_stepLenghtHipDisplaceRF = b_stepLenghtHipDisplaceLF;
	b_hipTrajectoryHeightRR   = b_hipTrajectoryHeightLR;
	b_hipTrajectoryHeightRF	  = b_hipTrajectoryHeightLF;
}

/*
@ -> THIGH sagitalLeg[0]
|
|
@ -> KNEE  sagitalLeg[1]
|
|
@ -> ANKLE sagitalLeg[2]
*/
void ARSRobotController::calculateAngleFromSagitalPosition(){
	/*************************DIREITA***************************/
	/** Para angulo da coxa baixa, cintura para o eixo Y**/
	Vector3f TK     = sagitalRightLeg[1] - sagitalRightLeg[0];
	Vector3f Y_AXIS = Vector3f(0,1,0);
	int thighAngle = 0;
	thighAngle = ARSAngleBetweenVectors(TK,Y_AXIS);// Lower thigh

	thighAngle = (360 - thighAngle*2)/2;

	if(TK.x < 0)
		thighAngle *= -1;

	/** Triangle relation para angulo do joelho **/
	float a = ARSDist2D(sagitalRightLeg[1], sagitalRightLeg[2]);
	float b = ARSDist2D(sagitalRightLeg[0], sagitalRightLeg[1]);
	float c = ARSDist2D(sagitalRightLeg[0], sagitalRightLeg[2]);

	float kneeAngle  = acosf((powf(a, 2.0f) + powf(b, 2.0f) - powf(c, 2.0f))/(2*a*b));    // KNEE ANGLE
	kneeAngle *= 180.0f / (float)pi;

	kneeAngle = (360 - kneeAngle*2)/2;

	int armAngle = 0;
	Vector3f TF = sagitalRightLeg[2] - sagitalRightLeg[0];
	armAngle    = ARSAngleBetweenVectors(TF,Y_AXIS);
	armAngle    = (360 - armAngle*2)/2;

	if(TF.x < 0)
		armAngle *= -1;

	servoCurrentAngle[ARS_LEFT_LOWER_SHOULDER] = (int)armAngle;
	servoCurrentAngle[ARS_RIGHT_THIGH] = (int)thighAngle;
	servoCurrentAngle[ARS_RIGHT_KNEE ] = (int)kneeAngle;

	if(assertByGiroscope){
			servoCurrentAngle[ARS_RIGHT_ANKLE] = (int)(kneeAngle - thighAngle +
		+footRotationRollOffset);
	}else{
	servoCurrentAngle[ARS_RIGHT_ANKLE] = (int)(kneeAngle - thighAngle +
		-footRotationRollOffset - 
		footRotationRollEquilibriumIterator);
	}
	/*************************ESQUERDA***************************/
	TK     = sagitalLeftLeg[1] - sagitalLeftLeg[0];
	Y_AXIS = Vector3f(0,1,0);
	thighAngle = 0;
	thighAngle = ARSAngleBetweenVectors(TK,Y_AXIS);// Lower thigh

	thighAngle = (360 - thighAngle*2)/2;

	if(TK.x < 0)
		thighAngle *= -1;

	/** Triangle relation para angulo do joelho **/
	a = ARSDist2D(sagitalLeftLeg[1], sagitalLeftLeg[2]);
	b = ARSDist2D(sagitalLeftLeg[0], sagitalLeftLeg[1]);
	c = ARSDist2D(sagitalLeftLeg[0], sagitalLeftLeg[2]);

	kneeAngle  = acosf((powf(a, 2.0f) + powf(b, 2.0f) - powf(c, 2.0f))/(2*a*b));    // KNEE ANGLE
	kneeAngle *= 180.0f / (float)pi;

	kneeAngle = (360 - kneeAngle*2)/2;

	armAngle = 0;
	TF = sagitalLeftLeg[2] - sagitalLeftLeg[0];
	armAngle    = ARSAngleBetweenVectors(TF,Y_AXIS);
	armAngle    = (360 - armAngle*2)/2;

	if(TF.x < 0)
		armAngle *= -1;

	servoCurrentAngle[ARS_RIGHT_LOWER_SHOULDER] = (int)armAngle;
	servoCurrentAngle[ARS_LEFT_THIGH] =  (int)thighAngle;
	servoCurrentAngle[ARS_LEFT_KNEE ] =  (int)kneeAngle;

	if(assertByGiroscope){
			servoCurrentAngle[ARS_LEFT_ANKLE] =   (int)(kneeAngle - thighAngle +
		+footRotationRollOffset);
	}else{
	servoCurrentAngle[ARS_LEFT_ANKLE] =   (int)(kneeAngle - thighAngle +
		-footRotationRollOffset - 
		footRotationRollEquilibriumIterator);
	}
}

void ARSRobotController::sendDataSagitalPlane(){
	ARS_IMoveSingleServo(ARS_RIGHT_ANKLE		  , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_RIGHT_ANKLE			], ARS_RIGHT_ANKLE			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_RIGHT_THIGH		  , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_RIGHT_THIGH			], ARS_RIGHT_THIGH			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_RIGHT_KNEE			  , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_RIGHT_KNEE			], ARS_RIGHT_KNEE			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_LEFT_ANKLE			  , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_LEFT_ANKLE			], ARS_LEFT_ANKLE			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_LEFT_THIGH			  , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_LEFT_THIGH			], ARS_LEFT_THIGH			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_LEFT_KNEE            , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_LEFT_KNEE		    ], ARS_LEFT_KNEE			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_LEFT_LOWER_SHOULDER  , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_LEFT_LOWER_SHOULDER  ], ARS_LEFT_LOWER_SHOULDER  ), 0, robotCom);
	ARS_IMoveSingleServo(ARS_RIGHT_LOWER_SHOULDER , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_RIGHT_LOWER_SHOULDER ], ARS_RIGHT_LOWER_SHOULDER ), 0, robotCom);
}

void ARSRobotController::sendDataFrontalPlane(){
	ARS_IMoveSingleServo(ARS_RIGHT_FOOT        , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_RIGHT_FOOT		  ], ARS_RIGHT_FOOT			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_RIGHT_UPPER_THIGH , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_RIGHT_UPPER_THIGH ], ARS_RIGHT_UPPER_THIGH  ), 0, robotCom);
	ARS_IMoveSingleServo(ARS_LEFT_FOOT         , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_LEFT_FOOT		  ], ARS_LEFT_FOOT			), 0, robotCom);
	ARS_IMoveSingleServo(ARS_LEFT_UPPER_THIGH  , ARS_DegreeToPulseRaw(servoCurrentAngle[ARS_LEFT_UPPER_THIGH  ], ARS_LEFT_UPPER_THIGH	), 0, robotCom);
}

void ARSRobotController::solveIk(){
	solkeIK_FABRIK(sagitalRightLeg , 3, Vector3f(sagitalRightFoot.x, sagitalRightFoot.y,0.0f));
	solkeIK_FABRIK(sagitalLeftLeg  , 3, Vector3f(sagitalLeftFoot .x, sagitalLeftFoot.y ,0.0f));
	solkeIK_FABRIK(frontalRightLeg , 2, Vector3f(frontalRightFoot.x, 0.0f			   ,0.0f));
	solkeIK_FABRIK(frontalLeftLeg  , 2, Vector3f(frontalLeftFoot .x, 0.0f			   ,0.0f));
}

void ARSRobotController::senusoidalTrajectory(){
	/*************************DIREITA***********************/
	// Computar trajetória e pegar pontos otimizados do PSO
	sagitalRightFoot.x =  -sagitalStepLenght * cosf((2.0f*(float)pi*(float)glfwGetTime()*timeFactor/1.0f)) + 0.0f;
	sagitalRightFoot.y =  sagitalStepHeight  * sinf((2.0f*(float)pi*(float)glfwGetTime()*timeFactor/1.0f)) + displacementY;

	if(sagitalRightFoot.y < displacementY)
		sagitalRightFoot.y = displacementY;

	/*************************ESQUERDA**********************/
	// Computar trajetória e pegar pontos otimizados do PSO
	sagitalLeftFoot.x =  -sagitalStepLenght * cosf(((2.0f*(float)pi*(float)glfwGetTime()*timeFactor + (float)pi)/1.0f)) + 0.0f;
	sagitalLeftFoot.y =   sagitalStepHeight * sinf(((2.0f*(float)pi*(float)glfwGetTime()*timeFactor + (float)pi)/1.0f)) + displacementY;

	if(sagitalLeftFoot.y < displacementY)
		sagitalLeftFoot.y = displacementY;

	/*************************DIREITA FRONTAL***********************/
	frontalRightFoot.x =  -frontalDisplacement * cosf(((2.0f*(float)pi*(float)glfwGetTime()*timeFactor)/1.0f));

	/*************************ESQUERDA FRONTAL**********************/
	frontalLeftFoot.x =  -frontalDisplacement * cosf(((2.0f*(float)pi*(float)glfwGetTime()*timeFactor)/1.0f));
}

void ARSRobotController::dynamicEquilibriumRotationCompensation(){
	/*************************DINAMIC PITCH EQUILIBRIUM**************/
	Vector3f lastYPR = giroscopeAccel->getLastYawPitchRollValue();
	Vector3f lastAceel = giroscopeAccel->getLastAccelValue();
	if(lastYPR.y < 1000 && lastYPR.y > -1000){
		if(assertByGiroscope)
		footRotationPitchOffset = lastYPR.y;
		else{
		float len = abs(baseFrontal - lastYPR.y);
		float norm = 0.0f;

		if(lastYPR.y < baseFrontal)
			norm = (1 - ARSNorm(lastYPR.y, baseFrontal - frontalTresh , baseFrontal));

		if(lastYPR.y > baseFrontal)
			norm = -1*ARSNorm(lastYPR.y, baseFrontal, baseFrontal + frontalTresh);

		footRotationPitchOffset = lastYPR.y * abs(norm);
		footRotationRollEquilibriumIterator = sagitalFootEquilibrium;
		}
	}

	if(lastYPR.z < 1000 && lastYPR.z > -1000){
		if(assertByGiroscope)
			footRotationRollOffset = lastYPR.z;
		else{

		float len = abs(baseSagital - lastYPR.z);
		float norm = 0.0f;

		if(lastYPR.z < baseSagital)
			norm = -1*(1 - ARSNorm(lastYPR.z, baseSagital - sagitalTresh , baseSagital));

		if(lastYPR.z > baseSagital)
			norm = ARSNorm(lastYPR.z, baseSagital, baseSagital + sagitalTresh);

		footRotationRollOffset = lastYPR.z * abs(norm);
		footRotationRollEquilibriumIterator = sagitalFootEquilibrium;
		}
	}
}

void ARSRobotController::configureBSplineLeftFoot(){
	/** Verifica limites **/
	if(b_stepLenghtDisplaceL >= b_sagitalStepLenghtL/2)
		b_stepLenghtDisplaceL = b_sagitalStepLenghtL/2 - 1;

	b_sagitalStepLenghtL = ARSClamp(b_sagitalStepLenghtL, minimumStepLenght, maximumStepLenght);
	b_sagitalStepHeightL = ARSClamp(b_sagitalStepHeightL, minimumStepHeight, maximumStepHeight);

	/** Configura largura do passo perna esquerda **/
	leftFootTrajectory.controlPoints[0].x = -b_sagitalStepLenghtL/2;
	leftFootTrajectory.controlPoints[3].x = b_sagitalStepLenghtL/2;

	/** Configura forma e altura do passo **/
	leftFootTrajectory.controlPoints[0].y = displacementY;
	leftFootTrajectory.controlPoints[1].x = b_stepLenghtDisplaceL - b_sagitalStepLenghtL/2;
	leftFootTrajectory.controlPoints[1].y = b_sagitalStepHeightL + displacementY;
	leftFootTrajectory.controlPoints[2].x = b_sagitalStepLenghtL/2 - b_stepLenghtDisplaceL;
	leftFootTrajectory.controlPoints[2].y = b_sagitalStepHeightL + displacementY;
	leftFootTrajectory.controlPoints[3].y = displacementY;

	// Configura cintura esquerda //
	/** Configura largura do passo perna esquerda **/
	leftFootHipTrajectory.controlPoints[0].x = -b_sagitalStepLenghtL/2;
	leftFootHipTrajectory.controlPoints[3].x = b_sagitalStepLenghtL/2;

	/** Configura forma e altura do passo **/
	leftFootHipTrajectory.controlPoints[0].y = displacementY;
	leftFootHipTrajectory.controlPoints[1].x = b_stepLenghtHipDisplaceLR - b_sagitalStepLenghtL/2;
	leftFootHipTrajectory.controlPoints[1].y = -b_hipTrajectoryHeightLR + displacementY;
	leftFootHipTrajectory.controlPoints[2].x = b_sagitalStepLenghtL/2 - b_stepLenghtHipDisplaceLF;
	leftFootHipTrajectory.controlPoints[2].y = -b_hipTrajectoryHeightLF + displacementY;
	leftFootHipTrajectory.controlPoints[3].y = displacementY;
}

void ARSRobotController::configureBSplineRightFoot(){
	/** Verifica limites **/
	if(b_stepLenghtDisplaceR >= b_sagitalStepLenghtR/2)
		b_stepLenghtDisplaceR = b_sagitalStepLenghtR/2 - 1;

	b_sagitalStepLenghtR = ARSClamp(b_sagitalStepLenghtR, minimumStepLenght, maximumStepLenght);
	b_sagitalStepHeightR = ARSClamp(b_sagitalStepHeightR, minimumStepHeight, maximumStepHeight);

	/** Configura largura do passo perna esquerda **/
	rightFootTrajectory.controlPoints[0].x = -b_sagitalStepLenghtR/2;
	rightFootTrajectory.controlPoints[3].x = b_sagitalStepLenghtR/2;

	/** Configura forma e altura do passo **/
	rightFootTrajectory.controlPoints[0].y = displacementY;
	rightFootTrajectory.controlPoints[1].x = b_stepLenghtDisplaceR - b_sagitalStepLenghtR/2;
	rightFootTrajectory.controlPoints[1].y = b_sagitalStepHeightR + displacementY;
	rightFootTrajectory.controlPoints[2].x = b_sagitalStepLenghtR/2 - b_stepLenghtDisplaceR;
	rightFootTrajectory.controlPoints[2].y = b_sagitalStepHeightR + displacementY;
	rightFootTrajectory.controlPoints[3].y = displacementY;

	// Configura cintura direita //
	/** Configura largura do passo perna esquerda **/
	rightFootHipTrajectory.controlPoints[0].x = -b_sagitalStepLenghtR/2;
	rightFootHipTrajectory.controlPoints[3].x = b_sagitalStepLenghtR/2;

	/** Configura forma e altura do passo **/
	rightFootHipTrajectory.controlPoints[0].y = displacementY;
	rightFootHipTrajectory.controlPoints[1].x = b_stepLenghtHipDisplaceRR - b_sagitalStepLenghtR/2;
	rightFootHipTrajectory.controlPoints[1].y = -b_hipTrajectoryHeightRR + displacementY;
	rightFootHipTrajectory.controlPoints[2].x = b_sagitalStepLenghtR/2 - b_stepLenghtHipDisplaceRF;
	rightFootHipTrajectory.controlPoints[2].y = -b_hipTrajectoryHeightRF + displacementY;
	rightFootHipTrajectory.controlPoints[3].y = displacementY;
}

void ARSRobotController::cubicBSplineTrajectory(){
	/** Configura todas as curvas interpoladas **/
	configureBSplineLeftFoot();
	configureBSplineRightFoot();

	//Parametrizar velocidade de deslocamento
	//float b_parametricRatioLNew = ARSNorm(b_parametricRatioL, 0, b_sagitalStepLenghtL);
	//float b_parametricRatioRNew = ARSNorm(b_parametricRatioL, 0, b_sagitalStepLenghtR);

	/*********************************************************************************************/
	/** Trajetoria do pe esquerdo **/
	b_parametricL += b_parametricRatioL * b_directionL * deltatime;

	if(b_parametricL > 1.0f){
		b_directionL *= -1.0f;
		b_parametricL = 1.0f;
	}

	if(b_parametricL < 0.0f){
		b_directionL *= -1.0f;
		b_parametricL = 0.0f;
	}

	/*********************************************************************************************/
	/** Trajetoria do pe direito **/
	b_parametricR += b_parametricRatioL * b_directionR * deltatime;

	if(b_parametricR > 1.0f){
		b_directionR *= -1.0f;
		b_parametricR = 1.0f;
	}

	if(b_parametricR < 0.0f){
		b_directionR *= -1.0f;
		b_parametricR = 0.0f;
	}
	/*********************************************************************************************/

	if(b_directionL > 0.0f)
		sagitalLeftFoot  = leftFootTrajectory.getPosition(b_parametricL, 3);
	else
		sagitalLeftFoot  = leftFootHipTrajectory.getPosition(b_parametricL, 3);

	if(b_directionR > 0.0f)
		sagitalRightFoot = rightFootTrajectory.getPosition(b_parametricR, 3);
	else
		sagitalRightFoot = rightFootHipTrajectory.getPosition(b_parametricR, 3);

	/** Configura parametrização de balanço frontal **/
	if(useDirectTransition){
	if(b_parametricR >= 1.0f){
		frontalRightFoot.x = b_frontalDisplacementR/2;
		frontalLeftFoot.x  = b_frontalDisplacementL/2;
	}

	if(b_directionR >= 1.0f){
		frontalRightFoot.x = -b_frontalDisplacementR/2;
		frontalLeftFoot.x  = -b_frontalDisplacementL/2;
	}
	}
	else{
	frontalRightFoot.x = ARSReverseNorm(b_parametricR, -b_frontalDisplacementR/2, b_frontalDisplacementR/2);
	frontalLeftFoot.x  = ARSReverseNorm(b_parametricR, -b_frontalDisplacementL/2, b_frontalDisplacementL/2);
	}
}

double meanD = 0.0;
int    itD = 0;

double trajSpline = 0.0;
double elisoidal  = 0.0;
double equilibri  = 0.0;
double angMap     = 0.0;
double FABRIK     = 0.0;
double dataSend   = 0.0;

void ARSRobotController::update(){
	double timeCheckStart = 0.0;
	double geral = glfwGetTime();

	//Configura mapeamento e ajustes
	configureDefaultServoPulse();
	configurePulsePerAngle();

		double timeCheckEnd = glfwGetTime();
		double execTime     = timeCheckEnd - timeCheckStart;

	if(mirrorPolinom)
		mirrorPolinomFunc();

	//Phase 0
	if(useElipsoid){
		timeCheckStart = glfwGetTime();
		senusoidalTrajectory();

		timeCheckEnd = glfwGetTime();
		 execTime     = timeCheckEnd - timeCheckStart;
		elisoidal += execTime;
		string toSend       = "--------Elipsoid-";
		toSend.append(std::to_string(elisoidal/(double)itD));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());

	}

	if(usePolinom){
		timeCheckStart = glfwGetTime();
		cubicBSplineTrajectory();
	
			timeCheckEnd = glfwGetTime();
		 execTime     = timeCheckEnd - timeCheckStart;
		trajSpline += execTime;
		string toSend       = "--------spline-";
		toSend.append(std::to_string(trajSpline/(double)itD));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());
	}

	//Phase 1
	if(useEquilibrium){
		timeCheckStart = glfwGetTime();
		dynamicEquilibriumRotationCompensation();

					timeCheckEnd = glfwGetTime();
		 execTime     = timeCheckEnd - timeCheckStart;
		equilibri += execTime;
		string toSend       = "--------equilibrium-";
		toSend.append(std::to_string(equilibri/(double)itD));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());
	}

	//Phase 2
	timeCheckStart = glfwGetTime();
	calculateAngleFromFrontalPosition();
	calculateAngleFromSagitalPosition();

						timeCheckEnd = glfwGetTime();
		 execTime     = timeCheckEnd - timeCheckStart;
		angMap += execTime;
		string toSend       = "--------angular-";
		toSend.append(std::to_string(angMap/(double)itD));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());

	//Phase 3
		timeCheckStart = glfwGetTime();
	solveIk();
							timeCheckEnd = glfwGetTime();
		 execTime     = timeCheckEnd - timeCheckStart;
		FABRIK += execTime;
		 toSend       = "--------IKSolver-";
		toSend.append(std::to_string(FABRIK/(double)itD));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());
	/**ENVIAR DADOS PARA PORTA SERIAL**/
		
		timeCheckStart = glfwGetTime();

	if(sendDataSagital)
		sendDataSagitalPlane();

	if(sendDataFrontal)
		sendDataFrontalPlane();

								timeCheckEnd = glfwGetTime();
		 execTime     = timeCheckEnd - timeCheckStart;
		dataSend += execTime;
		 toSend       = "--------DATA send-";
		toSend.append(std::to_string(dataSend/(double)itD));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());

		 timeCheckEnd = glfwGetTime();
		 execTime     = timeCheckEnd - geral;
							meanD += execTime;
		itD ++;

		 toSend       = "--------robotFusion-";
		toSend.append(std::to_string(meanD/(double)itD));
		toSend.append("-------------------\n");
		ARS_WriteToPipe(timeCheckTerminal, toSend.c_str());
}

void ARSRobotController::posUpdate(){

}