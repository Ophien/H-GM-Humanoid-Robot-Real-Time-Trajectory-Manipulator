#pragma once

#include <string>

using namespace std;
class Vector3f;

extern const double pi;

int ARSClamp(int value, int min, int max);
int ARSAngleBetweenVectors(Vector3f& A, Vector3f& B);
int ARSAngleFromLine();
float ARSClamp(float value, float min, float max);
float ARSDotProduct2D(Vector3f& A, Vector3f& B);
float ARSDotProduct3D(Vector3f& A, Vector3f& B);
float ARSDist2D(Vector3f& A, Vector3f& B);
float ARSDist3D(Vector3f& A, Vector3f& B);
float ARSNorm(float value, float min, float max);
float ARSReverseNorm(float parametric, float min, float max);

Vector3f operator* (float v, Vector3f& A);
Vector3f operator* (Vector3f& A, float v);
Vector3f operator+ (Vector3f& A, Vector3f& B);
Vector3f operator- (Vector3f& A, Vector3f& B);


class Vector3f{
public:
	Vector3f(){
		x = 0.0f;
		y = 0.0f;
		z = 0.0f;
		w = 0.0f;
	}

	Vector3f(float x, float y, float z){
		this -> x = x;
		this -> y = y;
		this -> z = z;
		w = 0.0f;
	}

		Vector3f(int x, int y, int z){
		this -> x = (float)x;
		this -> y = (float)y;
		this -> z = (float)z;
		w = 0.0f;
	}

	string toString(){
		string obj = "";
		obj.append("x: " + std::to_string(x) + "\ty: " + std::to_string(y) + "\tz: " + std::to_string(z) + "\n");
		return obj;
	}

	string toStringYPR(){
		string obj = "";
		obj.append("Yaw: " + std::to_string(z) + "\tPitch: " + std::to_string(y) + "\tRoll: " + std::to_string(z) + "\n");
		return obj;
	}

	float get2DLenght(){
		return sqrtf(powf(x, 2.0f) + powf(y, 2.0f));
	}

	float get3DLenght(){
		return sqrtf(powf(x, 2.0f) + powf(y, 2.0f) + powf(z, 2.0f));
	}

	void normalize(){
		float len = sqrtf(powf(x, 2.0f) + powf(y,2.0f) + powf(z,2.0f));
		x /= len;
		y /= len;
		z /= len;
	}

	float x, y, z, w;
};