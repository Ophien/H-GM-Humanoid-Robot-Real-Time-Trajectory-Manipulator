#include <string>
#include "ARS_Robotic.h"
#include "Serial.h"
#include "ARS_MathHelper.h"
#include "ARS_Global.h"

const int ARS_SERVO_CENTER = 1425;
int   * servoDefaultPulse    = new int	 [ARS_TOTAL_SERVOS];
int   * servoCurrentPulse    = new int	 [ARS_TOTAL_SERVOS];
int   * servoPreviousPulse   = new int	 [ARS_TOTAL_SERVOS]; 
int	  * servoCurrentAngle    = new int	 [ARS_TOTAL_SERVOS];
int   * parentMapping        = new int	 [ARS_TOTAL_SERVOS];
float * pulsePerAngle		 = new float [ARS_TOTAL_SERVOS];
bool  * servoReverse         = new bool	 [ARS_TOTAL_SERVOS];

Vector3f* sagitalRightLeg = new Vector3f[3];
Vector3f* sagitalLeftLeg  = new Vector3f[3];
Vector3f* frontalRightLeg = new Vector3f[2];
Vector3f* frontalLeftLeg  = new Vector3f[2];
Vector3f* sagitalRightArm = new Vector3f[2];
Vector3f* sagitalLeftArm  = new Vector3f[2];

int ARS_RIGHT_FOOT_V = 1360;
int ARS_RIGHT_ANKLE_V = 1617;
int ARS_RIGHT_KNEE_V = 1128;
int ARS_RIGHT_THIGH_V = 1400;
int ARS_RIGHT_UPPER_THIGH_V = 1386;
int ARS_RIGHT_LOWER_SHOULDER_V = 1337;
int ARS_RIGHT_HIGH_SHOULDER_V = 1012;
int ARS_RIGHT_ELBOW_V = 1105;

//ARS_COMMENT:Configurando lado esquerdo do biped
int ARS_LEFT_FOOT_V = 1350;
int ARS_LEFT_ANKLE_V = 1135;
int ARS_LEFT_KNEE_V = 1598;
int ARS_LEFT_THIGH_V = 1407;
int ARS_LEFT_UPPER_THIGH_V = 1398;
int ARS_LEFT_LOWER_SHOULDER_V = 1453;
int ARS_LEFT_HIGH_SHOULDER_V = 1826;
int ARS_LEFT_ELBOW_V = 1616;

bool	ARS_RIGHT_FOOT_R = false;
bool	ARS_RIGHT_ANKLE_R= false;
bool	ARS_RIGHT_KNEE_R = false;
bool	ARS_RIGHT_THIGH_R = false;
bool	ARS_RIGHT_UPPER_THIGH_R = false;
bool	ARS_RIGHT_LOWER_SHOULDER_R = false;
bool	ARS_RIGHT_HIGH_SHOULDER_R = false;
bool	ARS_RIGHT_ELBOW_R = false;

//ARS_COMMENT:Configurando lado esquerdo do biped
bool	ARS_LEFT_FOOT_R = true;
bool	ARS_LEFT_ANKLE_R = true;
bool	ARS_LEFT_KNEE_R = true;
bool	ARS_LEFT_THIGH_R = true;
bool	ARS_LEFT_UPPER_THIGH_R = true;
bool	ARS_LEFT_LOWER_SHOULDER_R = true;
bool	ARS_LEFT_HIGH_SHOULDER_R = false;
bool	ARS_LEFT_ELBOW_R = false;

float ARS_RIGHT_FOOT_P = 8.0f;
float ARS_RIGHT_ANKLE_P = 11.0f;
float ARS_RIGHT_KNEE_P = 11.0f;
float ARS_RIGHT_THIGH_P = 13.0f;
float ARS_RIGHT_UPPER_THIGH_P = 11.6f;
float ARS_RIGHT_LOWER_SHOULDER_P = 8.0f;
float ARS_RIGHT_HIGH_SHOULDER_P = 8.0f;
float ARS_RIGHT_ELBOW_P = 8.0f;

//ARS_COMMENT:Configurando lado esquerdo do biped
float ARS_LEFT_FOOT_P = 8.0f;
float ARS_LEFT_ANKLE_P = 11.0f;
float ARS_LEFT_KNEE_P = 11.0f;
float ARS_LEFT_THIGH_P = 13.0f;
float ARS_LEFT_UPPER_THIGH_P = 11.6f;
float ARS_LEFT_LOWER_SHOULDER_P = 8.0f;
float ARS_LEFT_HIGH_SHOULDER_P = 8.0f;
float ARS_LEFT_ELBOW_P = 8.0f;

void ARS_IMoveSingleServo(int servoID, int position, int completionTime, Serial* port){
	if(servoID >= ARS_TOTAL_SERVOS)
		ARS_TreatError(ARS_INVALID_EFFECTOR_ID);

	std::string data = "";

	//ARS_COMMENT:Buffers para armazenar string convertida dos valores desejados
	char servoIDChar       [16];
	char servoPositionChar [16];
	char completionTimeChar[16];

	//Normalizar pulso
	position = ARS_LimitServoPulse(position);

	//ARS_COMMENT:Conversão para string
	_itoa_s(servoID       ,servoIDChar       , 10);
	_itoa_s(position      ,servoPositionChar , 10);
	_itoa_s(completionTime,completionTimeChar, 10);

	//ARS_COMMENT:Montando comando
	data.append("#");
	data.append(servoIDChar);
	data.append("P");
	data.append(servoPositionChar);
	data.append("T");
	data.append(completionTimeChar);
	data.append("\r\n");

	//ARS_COMMENT:Enviando commando para porta serial
	char* cData = (char*)data.c_str();
	port->WriteData(cData, data.length());

	// Ajusta buffer de rotação anterior
	servoPreviousPulse[servoID] = servoCurrentPulse[servoID];

	// Ajusta buffer de rotação para novo pulso
	servoCurrentPulse[servoID] = position;

	//Beep(1500, 500);
	Sleep(completionTime);
}

int ARS_LimitServoPulse(int pulse){
	if(pulse < ARS_MIN_SERVO_PULSE)
		pulse = ARS_MIN_SERVO_PULSE;
	if(pulse > ARS_MAX_SERVO_PULSE)
		pulse = ARS_MAX_SERVO_PULSE;
	return pulse;
}

int ARS_NormalizeAngles(int angle){
	angle = angle % 360;

	//Garante intervalo positivo
	if (angle < 0) 
		angle += 360;

	//Garante intervalo negativo normalizado
	if(angle > 180)
		angle = angle - 360;

	return angle;
}

int  ARS_DegreeToPulseIncrement(int angle){
	// Normaliza angulo de entrada entre valores permitidos pelo servo
	angle = ARSClamp(angle, ARS_MAX_ANGLE_DECREMENT, ARS_MAX_ANGLE_INCREMENT);

	// Normaliza angulo
	int normalizedAngle = ARS_NormalizeAngles(angle);

	// Pulsos por angulo
	int pulse		    = (int)floor(ARS_PULSE_PER_ANGLE * normalizedAngle);

	return pulse;
}

int  ARS_DegreeToPulseRaw(int angle, int servoID){
	if(servoID >= ARS_TOTAL_SERVOS)
		ARS_TreatError(ARS_INVALID_EFFECTOR_ID);

	// Normaliza angulo de entrada entre valores permitidos pelo servo
	angle = ARSClamp(angle, ARS_MAX_ANGLE_DECREMENT, ARS_MAX_ANGLE_INCREMENT);

	// Normaliza angulo
	int normalizedAngle = ARS_NormalizeAngles(angle);

	// Pulsos por angulo
	int pulse		    = (int)floor(pulsePerAngle[servoID] * normalizedAngle);

	if(servoReverse[servoID])
		pulse = servoDefaultPulse[servoID] - pulse;
	else
		pulse = servoDefaultPulse[servoID] + pulse;

	return pulse;
}

void defaultStandInstance(Serial* serialPort){
	//ARS_COMMENT:Configurando lado direito do biped
	ARS_IMoveSingleServo(ARS_RIGHT_FOOT			  , 1360, 50		, serialPort);
	ARS_IMoveSingleServo(ARS_RIGHT_ANKLE		  , 1543, 50, serialPort);
	ARS_IMoveSingleServo(ARS_RIGHT_KNEE			  , 1128, 50, serialPort);
	ARS_IMoveSingleServo(ARS_RIGHT_THIGH		  , 1400, 50, serialPort);
	ARS_IMoveSingleServo(ARS_RIGHT_UPPER_THIGH    , 1386, 50, serialPort);
	ARS_IMoveSingleServo(ARS_RIGHT_LOWER_SHOULDER , 1337, 50, serialPort);
	ARS_IMoveSingleServo(ARS_RIGHT_HIGH_SHOULDER  , 1012, 50, serialPort);
	ARS_IMoveSingleServo(ARS_RIGHT_ELBOW		  , 1105, 50, serialPort);

	//ARS_COMMENT:Configurando lado esquerdo do biped
	ARS_IMoveSingleServo(ARS_LEFT_FOOT			 , 1350, 50, serialPort);
	ARS_IMoveSingleServo(ARS_LEFT_ANKLE			 , 1180 , 50, serialPort);
	ARS_IMoveSingleServo(ARS_LEFT_KNEE			 , 1598, 50, serialPort);
	ARS_IMoveSingleServo(ARS_LEFT_THIGH			 , 1407, 50, serialPort);
	ARS_IMoveSingleServo(ARS_LEFT_UPPER_THIGH	 , 1398, 50, serialPort);
	ARS_IMoveSingleServo(ARS_LEFT_LOWER_SHOULDER , 1453, 50, serialPort);
	ARS_IMoveSingleServo(ARS_LEFT_HIGH_SHOULDER	 , 1826, 50, serialPort);
	ARS_IMoveSingleServo(ARS_LEFT_ELBOW			 , 1616, 50, serialPort);

	//ARS_COMMENT:Configurando cabeça
	//ARS_IMoveSingleServo(30, 1375, 500, serialPort);
}

//Configure posição inicial e configuração do robô aqui
void defaultVirtualEffectorPosition(Vector3f* positionsArray){
	sagitalRightLeg[0] = Vector3f(0, 6 + 8, 0);
	sagitalRightLeg[1] = Vector3f(2, 6    , 0); // Deslocamento de 1 unidade para garantir flexionamento correto
	sagitalRightLeg[2] = Vector3f(0, 0    , 0);

	sagitalLeftLeg[0] = Vector3f(0, 6 + 8, 0);
	sagitalLeftLeg[1] = Vector3f(2, 6    , 0); // Deslocamento de 1 unidade para garantir flexionamento correto
	sagitalLeftLeg[2] = Vector3f(0, 0    , 0);

	frontalRightLeg[0] = Vector3f(0, 6 + 8, 0);
	frontalRightLeg[1] = Vector3f(0, 0, 0);

	frontalLeftLeg[0] = Vector3f(0, 6 + 8, 0);
	frontalLeftLeg[1] = Vector3f(0, 0, 0);

	sagitalLeftArm[0] = Vector3f(0, 6 + 8, 0);
	sagitalLeftArm[1] = Vector3f(0, 0, 0);

	sagitalRightArm[0] = Vector3f(0, 6 + 8, 0);
	sagitalRightArm[1] = Vector3f(0, 0, 0);
}

void configureDefaultServoPulse(){
	//ARS_COMMENT:Configurando lado direito do biped
	servoDefaultPulse[ARS_RIGHT_FOOT] = ARS_RIGHT_FOOT_V;
	servoDefaultPulse[ARS_RIGHT_ANKLE] = ARS_RIGHT_ANKLE_V;
	servoDefaultPulse[ARS_RIGHT_KNEE] = ARS_RIGHT_KNEE_V;
	servoDefaultPulse[ARS_RIGHT_THIGH] = ARS_RIGHT_THIGH_V;
	servoDefaultPulse[ARS_RIGHT_UPPER_THIGH] = ARS_RIGHT_UPPER_THIGH_V;
	servoDefaultPulse[ARS_RIGHT_LOWER_SHOULDER] = ARS_RIGHT_LOWER_SHOULDER_V;
	servoDefaultPulse[ARS_RIGHT_HIGH_SHOULDER] = ARS_RIGHT_HIGH_SHOULDER_V;
	servoDefaultPulse[ARS_RIGHT_ELBOW] = ARS_RIGHT_ELBOW_V;

	//ARS_COMMENT:Configurando lado esquerdo do biped
	servoDefaultPulse[ARS_LEFT_FOOT] = ARS_LEFT_FOOT_V;
	servoDefaultPulse[ARS_LEFT_ANKLE] = ARS_LEFT_ANKLE_V;
	servoDefaultPulse[ARS_LEFT_KNEE] = ARS_LEFT_KNEE_V;
	servoDefaultPulse[ARS_LEFT_THIGH] = ARS_LEFT_THIGH_V;
	servoDefaultPulse[ARS_LEFT_UPPER_THIGH] = ARS_LEFT_UPPER_THIGH_V;
	servoDefaultPulse[ARS_LEFT_LOWER_SHOULDER] = ARS_LEFT_LOWER_SHOULDER_V;
	servoDefaultPulse[ARS_LEFT_HIGH_SHOULDER] = ARS_LEFT_HIGH_SHOULDER_V;
	servoDefaultPulse[ARS_LEFT_ELBOW] = ARS_LEFT_ELBOW_V;

	servoReverse[ARS_RIGHT_FOOT]	= ARS_RIGHT_FOOT_R;
	servoReverse[ARS_RIGHT_ANKLE] = ARS_RIGHT_ANKLE_R;
	servoReverse[ARS_RIGHT_KNEE] = ARS_RIGHT_KNEE_R;
	servoReverse[ARS_RIGHT_THIGH] = ARS_RIGHT_THIGH_R;
	servoReverse[ARS_RIGHT_UPPER_THIGH] = ARS_RIGHT_UPPER_THIGH_R;
	servoReverse[ARS_RIGHT_LOWER_SHOULDER] = ARS_RIGHT_LOWER_SHOULDER_R;
	servoReverse[ARS_RIGHT_HIGH_SHOULDER] = ARS_RIGHT_HIGH_SHOULDER_R;
	servoReverse[ARS_RIGHT_ELBOW] = ARS_RIGHT_ELBOW_R;

	//ARS_COMMENT:Configurando lado esquerdo do biped
	servoReverse[ARS_LEFT_FOOT] = ARS_LEFT_FOOT_R;
	servoReverse[ARS_LEFT_ANKLE] = ARS_LEFT_ANKLE_R;
	servoReverse[ARS_LEFT_KNEE] = ARS_LEFT_KNEE_R;
	servoReverse[ARS_LEFT_THIGH] = ARS_LEFT_THIGH_R;
	servoReverse[ARS_LEFT_UPPER_THIGH] = ARS_LEFT_UPPER_THIGH_R;
	servoReverse[ARS_LEFT_LOWER_SHOULDER] = ARS_LEFT_LOWER_SHOULDER_R;
	servoReverse[ARS_LEFT_HIGH_SHOULDER] = ARS_LEFT_HIGH_SHOULDER_R;
	servoReverse[ARS_LEFT_ELBOW] = ARS_LEFT_ELBOW_R;
}

void configurePulsePerAngle(){
	pulsePerAngle[ARS_RIGHT_FOOT]	= ARS_RIGHT_FOOT_P;
	pulsePerAngle[ARS_RIGHT_ANKLE] = ARS_RIGHT_ANKLE_P;
	pulsePerAngle[ARS_RIGHT_KNEE] = ARS_RIGHT_KNEE_P;
	pulsePerAngle[ARS_RIGHT_THIGH] = ARS_RIGHT_THIGH_P;
	pulsePerAngle[ARS_RIGHT_UPPER_THIGH] = ARS_RIGHT_UPPER_THIGH_P;
	pulsePerAngle[ARS_RIGHT_LOWER_SHOULDER] = ARS_RIGHT_LOWER_SHOULDER_P;
	pulsePerAngle[ARS_RIGHT_HIGH_SHOULDER] = ARS_RIGHT_HIGH_SHOULDER_P;
	pulsePerAngle[ARS_RIGHT_ELBOW] = ARS_RIGHT_ELBOW_P;

	//ARS_COMMENT:Configurando lado esquerdo do biped
	pulsePerAngle[ARS_LEFT_FOOT] = ARS_LEFT_FOOT_P;
	pulsePerAngle[ARS_LEFT_ANKLE] = ARS_LEFT_ANKLE_P;
	pulsePerAngle[ARS_LEFT_KNEE] = ARS_LEFT_KNEE_P;
	pulsePerAngle[ARS_LEFT_THIGH] = ARS_LEFT_THIGH_P;
	pulsePerAngle[ARS_LEFT_UPPER_THIGH] = ARS_LEFT_UPPER_THIGH_P;
	pulsePerAngle[ARS_LEFT_LOWER_SHOULDER] = ARS_LEFT_LOWER_SHOULDER_P;
	pulsePerAngle[ARS_LEFT_HIGH_SHOULDER] = ARS_LEFT_HIGH_SHOULDER_P;
	pulsePerAngle[ARS_LEFT_ELBOW] = ARS_LEFT_ELBOW_P;
}


void solkeIK_FABRIK(Vector3f* sagital2DofLeg, int size, Vector3f& target){
	float* dists  = new float[size-1];
	float distSum = 0.0f;

	for(int i = 0; i < size - 1; i++){
		dists[i]     = ARSDist2D(sagital2DofLeg[i], sagital2DofLeg[i+1]);
		distSum     += dists[i];
	}

	float targetDist = ARSDist2D(sagital2DofLeg[0], target);

	if(targetDist > distSum){
		for(int i = 0; i < size-1; i++){
			float r_i       = ARSDist2D(target, sagital2DofLeg[i]);
			float lamb_i    = dists[i] / r_i;
			sagital2DofLeg[i+1] = ((1 - lamb_i) * sagital2DofLeg[i]) + (lamb_i * target);
		}
	}
	else{
		Vector3f b    = sagital2DofLeg[0];

		sagital2DofLeg[size-1] = target;

		for(int i = size-2; i >= 0; i--){
			float r_i    = ARSDist2D(sagital2DofLeg[i+1], sagital2DofLeg[i]);
			float lamb_i = dists[i] / r_i;
			sagital2DofLeg[i] = (1 - lamb_i) * sagital2DofLeg[i+1] + lamb_i * sagital2DofLeg[i];
		}

		sagital2DofLeg[0] = b;

		for(int i = 0; i < size-1; i++){
			float r_i    = ARSDist2D(sagital2DofLeg[i+1], sagital2DofLeg[i]);
			float lamb_i = dists[i] / r_i;
			sagital2DofLeg[i+1] = (1 - lamb_i) * sagital2DofLeg[i] + lamb_i * sagital2DofLeg[i+1];
		}
	}

	delete[]dists;
}