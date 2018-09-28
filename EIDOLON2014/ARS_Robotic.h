#pragma once

#include "Serial.h"
#include "ARS_MathHelper.h"

/** Dados do robo **/
#define ARS_SERVO_DEFAULT		 1500
#define ARS_MIN_SERVO_PULSE		 500
#define ARS_MAX_SERVO_PULSE		 2500
#define ARS_MAX_ANGLE			 180
#define ARS_MIN_ANGLE			 0
#define ARS_MAX_ANGLE_INCREMENT  110
#define ARS_MAX_ANGLE_DECREMENT -110

/** Calibrado individualmente **/
#define ARS_PULSE_PER_ANGLE		 6

/** Convenção de planos **/
//Sagital     = XY
//FRONTAL     = YZ
//TRANSVERSAL = XZ
#define ARS_TOTAL_SERVOS        32

#define ARS_LEFT_FOOT           16
#define ARS_LEFT_ANKLE          17
#define ARS_LEFT_KNEE           19
#define ARS_LEFT_THIGH          20
#define ARS_LEFT_UPPER_THIGH    22
#define ARS_LEFT_LOWER_SHOULDER 23
#define ARS_LEFT_HIGH_SHOULDER  25
#define ARS_LEFT_ELBOW          26

#define ARS_RIGHT_FOOT           0
#define ARS_RIGHT_ANKLE          1
#define ARS_RIGHT_KNEE           3
#define ARS_RIGHT_THIGH          4
#define ARS_RIGHT_UPPER_THIGH    6
#define ARS_RIGHT_LOWER_SHOULDER 7
#define ARS_RIGHT_HIGH_SHOULDER  9
#define ARS_RIGHT_ELBOW          10

/** Operações servo motor **/
void ARS_IMoveSingleServo(int servoID, int position, int completionTime, Serial* port);
int  ARS_LimitServoPulse (int pulse);
int  ARS_NormalizeAngles (int angle);

//Retorna pulso em forma de incremento, para rotaçao horaria e anti horaria, < 0 = anti horaria; > 0 = horaria
int  ARS_DegreeToPulseIncrement   (int angle);
int  ARS_DegreeToPulseRaw         (int angle, int servoID);

/** Gerenciamento robo e partes **/
extern const int ARS_SERVO_CENTER;
extern int   * servoDefaultPulse; 
extern int   * servoCurrentPulse;
extern int   * servoPreviousPulse;
extern int   * servoCurrentAngle;
extern int   * parentMapping;
extern float * pulsePerAngle;
extern bool  * servoReverse;

/** Mapeamento virtual **/
extern Vector3f* sagitalRightLeg;
extern Vector3f* sagitalLeftLeg;
extern Vector3f* frontalRightLeg;
extern Vector3f* frontalLeftLeg;
extern Vector3f* sagitalRightArm;
extern Vector3f* sagitalLeftArm;

/** Operações robo **/
void defaultStandInstance(Serial* serialPort);
void defaultVirtualEffectorPosition(Vector3f* positionsArray);
void configureDefaultServoPulse();
void configurePulsePerAngle();

/** Dinamica **/
void solkeIK_FABRIK(Vector3f* sagital2DofLeg, int size, Vector3f& target);

extern int ARS_RIGHT_FOOT_V;
extern int ARS_RIGHT_ANKLE_V;
extern int ARS_RIGHT_KNEE_V;
extern int ARS_RIGHT_THIGH_V;
extern int ARS_RIGHT_UPPER_THIGH_V;
extern int ARS_RIGHT_LOWER_SHOULDER_V;
extern int ARS_RIGHT_HIGH_SHOULDER_V;
extern int ARS_RIGHT_ELBOW_V;
extern int ARS_LEFT_FOOT_V;
extern int ARS_LEFT_ANKLE_V;
extern int ARS_LEFT_KNEE_V;
extern int ARS_LEFT_THIGH_V;
extern int ARS_LEFT_UPPER_THIGH_V;
extern int ARS_LEFT_LOWER_SHOULDER_V;
extern int ARS_LEFT_HIGH_SHOULDER_V;
extern int ARS_LEFT_ELBOW_V;

extern bool ARS_RIGHT_FOOT_R;
extern bool ARS_RIGHT_ANKLE_R;
extern bool ARS_RIGHT_KNEE_R;
extern bool ARS_RIGHT_THIGH_R;
extern bool ARS_RIGHT_UPPER_THIGH_R;
extern bool ARS_RIGHT_LOWER_SHOULDER_R;
extern bool ARS_RIGHT_HIGH_SHOULDER_R;
extern bool ARS_RIGHT_ELBOW_R;
extern bool ARS_LEFT_FOOT_R;
extern bool ARS_LEFT_ANKLE_R;
extern bool ARS_LEFT_KNEE_R;
extern bool ARS_LEFT_THIGH_R;
extern bool ARS_LEFT_UPPER_THIGH_R;
extern bool ARS_LEFT_LOWER_SHOULDER_R;
extern bool ARS_LEFT_HIGH_SHOULDER_R;
extern bool ARS_LEFT_ELBOW_R;

extern float ARS_RIGHT_FOOT_P;
extern float ARS_RIGHT_ANKLE_P;
extern float ARS_RIGHT_KNEE_P;
extern float ARS_RIGHT_THIGH_P;
extern float ARS_RIGHT_UPPER_THIGH_P;
extern float ARS_RIGHT_LOWER_SHOULDER_P;
extern float ARS_RIGHT_HIGH_SHOULDER_P;
extern float ARS_RIGHT_ELBOW_P;
extern float ARS_LEFT_FOOT_P;
extern float ARS_LEFT_ANKLE_P;
extern float ARS_LEFT_KNEE_P;
extern float ARS_LEFT_THIGH_P;
extern float ARS_LEFT_UPPER_THIGH_P;
extern float ARS_LEFT_LOWER_SHOULDER_P;
extern float ARS_LEFT_HIGH_SHOULDER_P;
extern float ARS_LEFT_ELBOW_P;