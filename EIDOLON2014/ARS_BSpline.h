#pragma once

#include "ARS_MathHelper.h"
#include <vector>

using namespace std;

#define T_MIN 0.0f
#define T_MAX 1.0f

class ARS_BSpline
{
public:
	ARS_BSpline(void);
	~ARS_BSpline(void);
	
	Vector3f getPosition(float parametrized, int curveDegree);
	
	vector<Vector3f> controlPoints;
	vector<float> knotVector;

private:
	Vector3f GetPoint(int i);
	float coxDeBoor(int i, int k, float parametrized);
};

