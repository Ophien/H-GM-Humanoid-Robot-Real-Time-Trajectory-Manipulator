#include "ARS_MathHelper.h"

const double pi     = 3.1415926535897932384626433832795;

int ARSClamp(int value, int min, int max){
	if(value < min) value = min;
	if(value > max) value = max;
	return value;
}

float ARSClamp(float value, float min, float max){
	if(value < min) value = min;
	if(value > max) value = max;
	return value;
}

int ARSAngleBetweenVectors(Vector3f& A, Vector3f& B){
	float lenghtA = A.get2DLenght();
	float lenghtB = B.get2DLenght();
	float dot     = ARSDotProduct2D(A,B);
	float angle   = dot / (lenghtA * lenghtB);
	float degree  = acosf(angle) * 180.0f / (float)pi;


	return (int)degree;
}

float ARSDotProduct2D(Vector3f& A, Vector3f& B){
	float dot = 0.0f;
	
	dot = A.x * B.x + A.y * B.y;
	
	return dot;
}

float ARSDotProduct3D(Vector3f& A, Vector3f& B){
	float dot = 0.0f;
	
	dot = A.x * B.x + A.y * B.y + A.z * B.z;
	
	return dot;
}

float ARSDist2D(Vector3f& A, Vector3f& B){
	float dist = 0;
	dist = sqrtf(pow(B.x - A.x, 2.0f) + pow(B.y - A.y, 2.0f)); 
	return dist;
}

float ARSDist3D(Vector3f& A, Vector3f& B){
	float dist = 0;
	dist = sqrtf(pow(B.x - A.x, 2.0f) + pow(B.y - A.y, 2.0f) + pow(B.z - A.z, 2.0f)); 
	return dist;
}

float ARSNorm(float value, float min, float max){
	float norm = (value - min) / (max - min);
	return norm;
}

float ARSReverseNorm(float parametric, float min, float max){
	float reverse = parametric*(max-min) + min;
	return reverse;
}

Vector3f operator- (Vector3f& A, Vector3f& B){
	Vector3f newVec;
	newVec.x = A.x - B.x;
	newVec.y = A.y - B.y;
	newVec.z = A.z - B.z;
	return newVec;
}

Vector3f operator+ (Vector3f& A, Vector3f& B){
	Vector3f newVec;
	newVec.x = A.x + B.x;
	newVec.y = A.y + B.y;
	newVec.z = A.z + B.z;
	return newVec;
}

Vector3f operator* (float    v, Vector3f& A){
	Vector3f newVec;
	newVec.x = A.x * v;
	newVec.y = A.y * v;
	newVec.z = A.z * v;
	return newVec;
}

Vector3f operator* (Vector3f& A, float   v){
	Vector3f newVec;
	newVec.x = A.x * v;
	newVec.y = A.y * v;
	newVec.z = A.z * v;
	return newVec;
}

