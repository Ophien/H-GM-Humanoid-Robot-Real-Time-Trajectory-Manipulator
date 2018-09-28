#pragma once

#include "ARSBehavior.h"
#include "ARSArduinoCom.h"
#include <vector>
#include "ARS_MathHelper.h"
#include "ARS_Global.h"

#define INICIAL 0
#define GET_X   1
#define GET_Y   2
#define GET_Z   3
#define STAT_GIRO 4
#define GET_YA 5
#define GET_P 6
#define GET_R 7

#define ACCEL_X_OFFSET 0.0f
#define ACCEL_Y_OFFSET 0.15f
#define ACCEL_Z_OFFSET -0.99f

class ARSArduinoLAccel : public ARSBehavior 
{
public:
	ARSArduinoLAccel(ARSArduinoCom* comBehavior);
	~ARSArduinoLAccel(void);

	void initialize();
	void preUpdate();
	void posUpdate();
	void update();
	void draw();

	Vector3f getLastAccelValue(){
		if(availableAccel.size() > 0)
			return availableAccel.back();
		return Vector3f(0,0,0);
	}

	Vector3f getLastYawPitchRollValue(){
		if(availableYawPitchRoll.size() > 0)
			return availableYawPitchRoll.back();
		return Vector3f(0,0,0);
	}

private:
	//Para controlar o parser dos dados do arduino
	int curState;
	string x;
	string y;
	string z;

	ARSArduinoCom*   comBehavior;
	vector<Vector3f> availableAccel;
	vector<Vector3f> availableYawPitchRoll;

	int accelConsole;
	int pitchYawRollConsole;
};

