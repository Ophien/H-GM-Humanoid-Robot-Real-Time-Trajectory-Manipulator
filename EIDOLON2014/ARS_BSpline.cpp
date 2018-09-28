#include "ARS_BSpline.h"
#include "ARS_Global.h"

ARS_BSpline::ARS_BSpline(void)
{
	//Vetor de knot basico para interpolação uniforme aberta de grau 3
	knotVector.push_back(0.0f);
	knotVector.push_back(0.0f);
	knotVector.push_back(0.0f);
	knotVector.push_back(0.0f);//1.0f/3.0f);
	knotVector.push_back(1.0f);//(2.0f/3.0f);
	knotVector.push_back(1.0f);//(3.0f/3.0f);
	knotVector.push_back(1.0f);//(3.0f/3.0f);
	knotVector.push_back(1.0f);//(3.0f/3.0f);
}

ARS_BSpline::~ARS_BSpline(void)
{
}

float ARS_BSpline::coxDeBoor(int i, int k, float parametrized){
	//Condição de parada
	if(k == 0){
		if(knotVector[i] <= parametrized && knotVector[i+1] >= parametrized)
			return 1.0f;
		else
			return 0.0f;
	}

	//Recurção

	float partA = 0.0f;
	float subA = (knotVector[i+k] - knotVector[i]);

	if(subA>0){
		partA = (parametrized - knotVector[i])/subA;
		partA = partA * coxDeBoor(i, k-1, parametrized);
	}

	float partB = 0.0f;
	float subB = (knotVector[i+k+1] - knotVector[i+1]);
	
	if(subB>0.0f){
		partB = (knotVector[i+k+1] - parametrized)/subB;
		partB = partB * coxDeBoor(i+1, k-1, parametrized);
	}

	float result = partA + partB;

	return result;
}

Vector3f ARS_BSpline::getPosition(float parametrized, int curveDegree){
	Vector3f point;

	//if(controlPoints.size() != knotVector.size())
	//	return point;

	if(curveDegree < 2)
		curveDegree = 2;

	if(curveDegree > controlPoints.size())
		curveDegree = controlPoints.size();

	if(parametrized == 1.0f){
		printf("");
	}

	for(int i = 0; i < controlPoints.size(); i++){
		float cox = coxDeBoor(i,curveDegree,parametrized);
		point = point + controlPoints[i]*cox;
	}

	return point;
}