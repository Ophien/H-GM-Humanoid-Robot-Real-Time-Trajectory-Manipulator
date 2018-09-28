#pragma once

#include <string>

#include "ARS_Global.h"
#include "ARSBehavior.h"
#include "ARS_Robotic.h"
#include "Serial.h"
#include "ARSArduinoLAccel.h"

#include "GLFW\glfw3.h"
#include <gl\GLU.h>
#include "ARS_BSpline.h"

class ARSRobotController : public ARSBehavior
{
public:
	ARSRobotController (char* comPort, ARSArduinoLAccel* giroscopeAccelModule);
	~ARSRobotController(void);

	void initialize();
	void preUpdate();
	void update();
	void posUpdate();
	void draw();

	void calculateAngleFromFrontalPosition();
	void calculateAngleFromSagitalPosition();
	void sendDataSagitalPlane();
	void sendDataFrontalPlane();
	void dynamicEquilibriumRotationCompensation();
	void senusoidalTrajectory();
	void configureBSplineLeftFoot();
	void configureBSplineRightFoot();
	void cubicBSplineTrajectory();
	void solveIk();
	void mirrorPolinomFunc();

	Serial			* robotCom;
	char			* port;
	GLUquadricObj	* sphere;
	ARSArduinoLAccel* giroscopeAccel;
	int rightFootPositionConsole;

	/** Global controllers **/
	float displacementY;
	float minimumStepHeight;
	float minimumStepLenght;
	float maximumStepHeight;
	float maximumStepLenght;
	
	Vector3f sagitalLeftFoot;
	Vector3f sagitalRightFoot;
	Vector3f sagitalLeftArm;
	Vector3f sagitalRightArm;
	Vector3f frontalLeftFoot;
	Vector3f frontalRightFoot;

	/** Controle de curva básica senoidal **/
	float timeFactor;
	float frontalDisplacement;
	float sagitalStepLenght;
	float sagitalStepHeight;
	
	/** Rotation compensation equilibrium **/
	float footRotationPitchOffset;
	float footRotationPitchThreshold;
	float footRotationRollThreshold;
	float footRotationRollOffset;
	float footRotationRollEquilibriumIterator;
	float sagitalFootEquilibrium;

	/** B-Spline trajectory params **/
	ARS_BSpline leftFootTrajectory;
	ARS_BSpline leftFootHipTrajectory;
	ARS_BSpline rightFootTrajectory;
	ARS_BSpline rightFootHipTrajectory;

	float baseSagital;
	float baseFrontal;
	float sagitalTresh;
	float frontalTresh;

	float b_sagitalStepLenghtL;
	float b_sagitalStepLenghtR;
	float b_sagitalStepHeightL;
	float b_sagitalStepHeightR;
	float b_frontalDisplacementL;
	float b_frontalDisplacementR;
	float b_hipTrajectoryHeightLR;
	float b_hipTrajectoryHeightLF;
	float b_hipTrajectoryHeightRR;
	float b_hipTrajectoryHeightRF;
	float b_directionL;
	float b_directionR;
	float b_parametricL;
	float b_parametricR;
	float b_parametricRatioL;
	float b_parametricRatioR;
	float b_stepLenghtDisplaceL;
	float b_stepLenghtDisplaceR;
	float b_stepLenghtHipDisplaceLR;
	float b_stepLenghtHipDisplaceLF;
	float b_stepLenghtHipDisplaceRR;
	float b_stepLenghtHipDisplaceRF;
	float b_frontalLfootrotratio;
	float b_frontalRfootrotratio;

	float b_giroscopeAssertingRateL;
	float b_giroscopeAssertingRateR;

	bool assertByGiroscope;
	bool useDirectTransition;
	bool sendDataSagital;
	bool sendDataFrontal;
	bool useEquilibrium;
	bool usePolinom;
	bool useElipsoid;
	bool mirrorPolinom;
	bool invertFrontal;
};

