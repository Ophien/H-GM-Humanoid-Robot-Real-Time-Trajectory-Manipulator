#include "ARS_TweekControl.h"color='0 0 0' alpha=255
#include "AntTweakBar.h"
#include "ARS_Global.h"
#include "ARS_Robotic.h"

ARSRobotController* robot = NULL;

void createAllBars(){
	TwDefine(" TW_HELP visible=false ");

	//LeftLeg
	TwBar *leftLeg;
	leftLeg = TwNewBar("LeftLeg"); 
	TwDefine("LeftLeg alwaystop=true color='0 0 0' alpha=255 size='200 210' position='0 0' movable=false resizable=false");

	TwAddVarRW(leftLeg, "step lenght"  , TW_TYPE_FLOAT, &robot->b_sagitalStepLenghtL , " min=0 max=10 step=0.01 group=Sagital label='Step lenght' ");
	TwAddVarRW(leftLeg, "step height"  , TW_TYPE_FLOAT, &robot->b_sagitalStepHeightL , " min=0 max=10 step=0.01 group=Sagital label='Step height' ");
	TwAddVarRW(leftLeg, "step displace", TW_TYPE_FLOAT, &robot->b_stepLenghtDisplaceL, " min=0 max=10 step=0.01 group=Sagital label='Step displace' ");

	TwAddVarRW(leftLeg, "hip lenght"        , TW_TYPE_FLOAT, &robot->b_sagitalStepLenghtL     , " min=0 max=10 step=0.01 group=Sagital label='Hip lenght' ");
	TwAddVarRW(leftLeg, "hip height rear"   , TW_TYPE_FLOAT, &robot->b_hipTrajectoryHeightLR  , " min=0 max=10 step=0.01 group=Sagital label='Hip height rear' ");
	TwAddVarRW(leftLeg, "hip height front"  , TW_TYPE_FLOAT, &robot->b_hipTrajectoryHeightLF  , " min=0 max=10 step=0.01 group=Sagital label='Hip height front' ");
	TwAddVarRW(leftLeg, "hip displace rear" , TW_TYPE_FLOAT, &robot->b_stepLenghtHipDisplaceLR, " min=0 max=10 step=0.01 group=Sagital label='Hip displace rear' ");
	TwAddVarRW(leftLeg, "hip displace front", TW_TYPE_FLOAT, &robot->b_stepLenghtHipDisplaceLF, " min=0 max=10 step=0.01 group=Sagital label='Hip displace front' ");

	TwAddVarRW(leftLeg, "mirror do right leg", TW_TYPE_BOOL8, &robot->mirrorPolinom, "group=mirroring label='mirror do right leg'");

	//Right
	TwBar *rightLeg;
	rightLeg = TwNewBar("RightLeg");
	TwDefine("RightLeg alwaystop=true color='0 0 0' alpha=255 size='200 210' position='200 0' movable=false resizable=false");

	TwAddVarRW(rightLeg, "step lenght"  , TW_TYPE_FLOAT, &robot->b_sagitalStepLenghtR , " min=0 max=10 step=0.01 group=Sagital label='Step lenght' ");
	TwAddVarRW(rightLeg, "step height"  , TW_TYPE_FLOAT, &robot->b_sagitalStepHeightR , " min=0 max=10 step=0.01 group=Sagital label='Step height' ");
	TwAddVarRW(rightLeg, "step displace", TW_TYPE_FLOAT, &robot->b_stepLenghtDisplaceR, " min=0 max=10 step=0.01 group=Sagital label='Step displace' ");

	TwAddVarRW(rightLeg, "hip lenght"        , TW_TYPE_FLOAT, &robot->b_sagitalStepLenghtR     , " min=0 max=10 step=0.01 group=Sagital label='Hip lenght' ");
	TwAddVarRW(rightLeg, "hip height rear"   , TW_TYPE_FLOAT, &robot->b_hipTrajectoryHeightRR  , " min=0 max=10 step=0.01 group=Sagital label='Hip height rear' ");
	TwAddVarRW(rightLeg, "hip height front"  , TW_TYPE_FLOAT, &robot->b_hipTrajectoryHeightRF  , " min=0 max=10 step=0.01 group=Sagital label='Hip height front' ");
	TwAddVarRW(rightLeg, "hip displace rear" , TW_TYPE_FLOAT, &robot->b_stepLenghtHipDisplaceRR, " min=0 max=10 step=0.01 group=Sagital label='Hip displace rear' ");
	TwAddVarRW(rightLeg, "hip displace front", TW_TYPE_FLOAT, &robot->b_stepLenghtHipDisplaceRF, " min=0 max=10 step=0.01 group=Sagital label='Hip displace front' ");

	TwBar *elipsoid;
	elipsoid = TwNewBar("elipsoid");
	TwDefine("elipsoid alwaystop=true color='0 0 0' alpha=255 size='200 210' position='0 210' movable=false resizable=false");

	TwAddVarRW(elipsoid, "step lenght"				, TW_TYPE_FLOAT, &robot->sagitalStepLenght	, "min=0 max=5 step=0.01 group=Sagital label='step lenght'");
	TwAddVarRW(elipsoid, "step height"				, TW_TYPE_FLOAT, &robot->sagitalStepHeight	, "min=0 max=5 step=0.01 group=Sagital label='step height'");
	TwAddVarRW(elipsoid, "frontal displacement"		, TW_TYPE_FLOAT, &robot->frontalDisplacement, "min=0 max=5 step=0.01 group=Frontal label='frontal displacement'");
	TwAddVarRW(elipsoid, "time factor"				, TW_TYPE_FLOAT, &robot->timeFactor			, "min=0 max=5 step=0.01 group=Time    label='time factor'");

	TwBar *frontalDisplace;
	frontalDisplace = TwNewBar("FrontalandGeneral");
	TwDefine("FrontalandGeneral alwaystop=true color='0 0 0' alpha=255 size='200 210' position='400 0' movable=false resizable=false");

	TwAddVarRW(frontalDisplace, "displace left leg" , TW_TYPE_FLOAT, &robot->b_frontalDisplacementL   , " min=0 max=10 step=0.01 group=Displace label='left leg' ");
	TwAddVarRW(frontalDisplace, "displace right leg", TW_TYPE_FLOAT, &robot->b_frontalDisplacementR   , " min=0 max=10 step=0.01 group=Displace label='right leg' ");

	TwAddVarRW(frontalDisplace, "left foot ratio" , TW_TYPE_FLOAT, &robot->b_frontalLfootrotratio   , " min=0 max=10 step=0.01 group=Displace label='left foot ratio' ");
	TwAddVarRW(frontalDisplace, "right foot ratio", TW_TYPE_FLOAT, &robot->b_frontalRfootrotratio   , " min=0 max=10 step=0.01 group=Displace label='right foot ratio' ");

	TwAddVarRW(frontalDisplace, "Y displacement"	, TW_TYPE_FLOAT  , &robot->displacementY		   , " min=0 max=50 step=0.01 group=Speed label='Y displacement' ");
	TwAddVarRW(frontalDisplace, "Giroscope Assert Rate Left"	, TW_TYPE_FLOAT  , &robot->b_giroscopeAssertingRateL		   , " min=0 max=50 step=0.01 group=Speed label='GAR Left' ");
	TwAddVarRW(frontalDisplace, "Giroscope Assert Rate Right"	, TW_TYPE_FLOAT  , &robot->b_giroscopeAssertingRateR		   , " min=0 max=50 step=0.01 group=Speed label='GAR Right' ");
	TwAddVarRW(frontalDisplace, "step speed X-axis", TW_TYPE_FLOAT  , &robot->b_parametricRatioL		   , " min=0 max=50 step=0.01 group=Speed label='step speed X-axis' ");

	TwAddButton(frontalDisplace, "reset parametric", resetStepParameter, NULL, " label='reset parametric' group='Parameters' ");
	TwAddButton(frontalDisplace, "to stand", resetToStandPosition, NULL, " label='to stand' group='Parameters' ");

	TwBar *dataopt;
	dataopt = TwNewBar("DataOpt");
	TwDefine("DataOpt alwaystop=true color='0 0 0' alpha=255 size='200 210' position='200 210' movable=false resizable=false");

	TwAddVarRW(dataopt, "send sagital data", TW_TYPE_BOOL8, &robot->sendDataSagital, "group=Data label='send sagital data'");
	TwAddVarRW(dataopt, "send frontal data", TW_TYPE_BOOL8, &robot->sendDataFrontal, "group=Data label='send frontal data'");
	TwAddVarRW(dataopt, "use equilibrium", TW_TYPE_BOOL8, &robot->useEquilibrium, "group=Data label='use equilibrium'");
	TwAddVarRW(dataopt, "use polinom", TW_TYPE_BOOL8, &robot->usePolinom, "group=Data label='use polinom'");
	TwAddVarRW(dataopt, "use elipsoid", TW_TYPE_BOOL8, &robot->useElipsoid, "group=Data label='use elipsoid'");
	TwAddVarRW(dataopt, "invert frontal data", TW_TYPE_BOOL8, &robot->invertFrontal, "group=Data label='invert frontal data'");
	TwAddVarRW(dataopt, "frontal direct transition", TW_TYPE_BOOL8, &robot->useDirectTransition, "group=Data label='frontal direct transition'");
	TwAddVarRW(dataopt, "assert by giroscope", TW_TYPE_BOOL8, &robot->assertByGiroscope, "group=Data label='assert by giroscope'");
	TwAddVarRW(dataopt, "use SGPSD", TW_TYPE_BOOL8, &sendToConsole, "group=Data label='use SGPSD'");

	TwBar *equiParam;
	equiParam = TwNewBar("equiParam");
	TwDefine("equiParam alwaystop=true color='0 0 0' alpha=255 size='200 210' position='400 210' movable=false resizable=false");

	TwAddVarRW(equiParam, "baseSagital", TW_TYPE_FLOAT, &robot->baseSagital, " min=-100 max=100 step=0.01 group=sagital label='baseSagital'");
	TwAddVarRW(equiParam, "sagitalTresh", TW_TYPE_FLOAT, &robot->sagitalTresh, " min=-100 max=100 step=0.01 group=sagital label='sagitalTresh'");
	TwAddVarRW(equiParam, "baseFrontal", TW_TYPE_FLOAT, &robot->baseFrontal, " min=-100 max=100 step=0.01 group=frontal label='baseFrontal'");
	TwAddVarRW(equiParam, "frontalTresh", TW_TYPE_FLOAT, &robot->frontalTresh, " min=-100 max=100 step=0.01 group=frontal label='frontalTresh'");

	TwBar *servoDefault;
	servoDefault = TwNewBar("ServoDefault");
	TwDefine("ServoDefault alwaystop=true color='0 0 0' alpha=255 size='400 850' position='1520 0' movable=false resizable=false");
	TwAddVarRW(servoDefault, "ARS_RIGHT_FOOT_V", TW_TYPE_INT16, &ARS_RIGHT_FOOT_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_FOOT_V'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_ANKLE_V", TW_TYPE_INT16, &ARS_RIGHT_ANKLE_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_ANKLE_V'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_KNEE_V", TW_TYPE_INT16, &ARS_RIGHT_KNEE_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_KNEE_V'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_THIGH_V", TW_TYPE_INT16, &ARS_RIGHT_THIGH_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_THIGH_V'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_UPPER_THIGH_V", TW_TYPE_INT16, &ARS_RIGHT_UPPER_THIGH_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_UPPER_THIGH_V'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_LOWER_SHOULDER_V", TW_TYPE_INT16, &ARS_RIGHT_LOWER_SHOULDER_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_LOWER_SHOULDER_V'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_HIGH_SHOULDER_V", TW_TYPE_INT16, &ARS_RIGHT_HIGH_SHOULDER_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_HIGH_SHOULDER_V'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_ELBOW_V", TW_TYPE_INT16, &ARS_RIGHT_ELBOW_V, "min=500 max=2500  group=servoR   label='ARS_RIGHT_ELBOW_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_FOOT_V", TW_TYPE_INT16, &ARS_LEFT_FOOT_V, "min=500 max=2500  group=servoL   label='ARS_LEFT_FOOT_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_ANKLE_V", TW_TYPE_INT16, &ARS_LEFT_ANKLE_V, "min=500 max=2500  group=servoL   label='ARS_LEFT_ANKLE_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_KNEE_V", TW_TYPE_INT16, &ARS_LEFT_KNEE_V, "min=500 max=2500  group=servoL   label='ARS_LEFT_KNEE_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_THIGH_V", TW_TYPE_INT16, &ARS_LEFT_THIGH_V, "min=500 max=2500  group=servoL  label='ARS_LEFT_THIGH_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_UPPER_THIGH_V", TW_TYPE_INT16, &ARS_LEFT_UPPER_THIGH_V, "min=500 max=2500  group=servoL   label='ARS_LEFT_UPPER_THIGH_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_LOWER_SHOULDER_V", TW_TYPE_INT16, &ARS_LEFT_LOWER_SHOULDER_V, "min=500 max=2500  group=servoL   label='ARS_LEFT_LOWER_SHOULDER_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_HIGH_SHOULDER_V", TW_TYPE_INT16, &ARS_LEFT_HIGH_SHOULDER_V, "min=500 max=2500  group=servoL   label='ARS_LEFT_HIGH_SHOULDER_V'");
	TwAddVarRW(servoDefault, "ARS_LEFT_ELBOW_V", TW_TYPE_INT16, &ARS_LEFT_ELBOW_V, "min=500 max=2500  group=servoL   label='ARS_LEFT_ELBOW_V'");

	TwAddVarRW(servoDefault, "ARS_RIGHT_FOOT_P", TW_TYPE_FLOAT, &ARS_RIGHT_FOOT_P, " group=pulseR min=0 max=25 step=0.01  label='ARS_RIGHT_FOOT_P'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_ANKLE_P", TW_TYPE_FLOAT, &ARS_RIGHT_ANKLE_P, "  group=pulseR min=0 max=25 step=0.01  label='ARS_RIGHT_ANKLE_P'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_KNEE_P", TW_TYPE_FLOAT, &ARS_RIGHT_KNEE_P, " group=pulseR min=0 max=25 step=0.01  label='ARS_RIGHT_KNEE_P'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_THIGH_P", TW_TYPE_FLOAT, &ARS_RIGHT_THIGH_P, "  group=pulseR min=0 max=25 step=0.01  label='ARS_RIGHT_THIGH_P'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_UPPER_THIGH_P", TW_TYPE_FLOAT, &ARS_RIGHT_UPPER_THIGH_P, "  group=pulseR  min=0 max=25 step=0.01 label='ARS_RIGHT_UPPER_THIGH_P'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_LOWER_SHOULDER_P", TW_TYPE_FLOAT, &ARS_RIGHT_LOWER_SHOULDER_P, "  group=pulseR  min=0 max=25 step=0.01 label='ARS_RIGHT_LOWER_SHOULDER_P'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_HIGH_SHOULDER_P", TW_TYPE_FLOAT, &ARS_RIGHT_HIGH_SHOULDER_P, " group=pulseR min=0 max=25 step=0.01  label='ARS_RIGHT_HIGH_SHOULDER_P'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_ELBOW_P", TW_TYPE_FLOAT, &ARS_RIGHT_ELBOW_P, "  group=pulseR min=0 max=25 step=0.01  label='ARS_RIGHT_ELBOW_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_FOOT_P", TW_TYPE_FLOAT, &ARS_LEFT_FOOT_P, "  group=pulseL min=0 max=25 step=0.01  label='ARS_LEFT_FOOT_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_ANKLE_P", TW_TYPE_FLOAT, &ARS_LEFT_ANKLE_P, "  group=pulseL min=0 max=25 step=0.01  label='ARS_LEFT_ANKLE_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_KNEE_P", TW_TYPE_FLOAT, &ARS_LEFT_KNEE_P, "  group=pulseL min=0 max=25 step=0.01  label='ARS_LEFT_KNEE_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_THIGH_P", TW_TYPE_FLOAT, &ARS_LEFT_THIGH_P, " group=pulseL min=0 max=25 step=0.01  label='ARS_LEFT_THIGH_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_UPPER_THIGH_P", TW_TYPE_FLOAT, &ARS_LEFT_UPPER_THIGH_P, "  group=pulseL min=0 max=25 step=0.01  label='ARS_LEFT_UPPER_THIGH_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_LOWER_SHOULDER_P", TW_TYPE_FLOAT, &ARS_LEFT_LOWER_SHOULDER_P, " group=pulseL min=0 max=25 step=0.01  label='ARS_LEFT_LOWER_SHOULDER_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_HIGH_SHOULDER_P", TW_TYPE_FLOAT, &ARS_LEFT_HIGH_SHOULDER_P, "  group=pulseL  min=0 max=25 step=0.01 label='ARS_LEFT_HIGH_SHOULDER_P'");
	TwAddVarRW(servoDefault, "ARS_LEFT_ELBOW_P", TW_TYPE_FLOAT, &ARS_LEFT_ELBOW_P, " group=pulseL  min=0 max=25 step=0.01 label='ARS_LEFT_ELBOW_P'");

	TwAddVarRW(servoDefault, "ARS_RIGHT_FOOT_R", TW_TYPE_BOOL8, &ARS_RIGHT_FOOT_R, " group=reverseR   label='ARS_RIGHT_FOOT_R'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_ANKLE_R", TW_TYPE_BOOL8, &ARS_RIGHT_ANKLE_R, "  group=reverseR   label='ARS_RIGHT_ANKLE_R'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_KNEE_R", TW_TYPE_BOOL8, &ARS_RIGHT_KNEE_R, " group=reverseR   label='ARS_RIGHT_KNEE_R'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_THIGH_R", TW_TYPE_BOOL8, &ARS_RIGHT_THIGH_R, "  group=reverseR   label='ARS_RIGHT_THIGH_R'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_UPPER_THIGH_R", TW_TYPE_BOOL8, &ARS_RIGHT_UPPER_THIGH_R, "  group=reverseR   label='ARS_RIGHT_UPPER_THIGH_R'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_LOWER_SHOULDER_R", TW_TYPE_BOOL8, &ARS_RIGHT_LOWER_SHOULDER_R, "  group=reverseR   label='ARS_RIGHT_LOWER_SHOULDER_R'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_HIGH_SHOULDER_R", TW_TYPE_BOOL8, &ARS_RIGHT_HIGH_SHOULDER_R, " group=reverseR   label='ARS_RIGHT_HIGH_SHOULDER_R'");
	TwAddVarRW(servoDefault, "ARS_RIGHT_ELBOW_R", TW_TYPE_BOOL8, &ARS_RIGHT_ELBOW_R, "  group=reverseR   label='ARS_RIGHT_ELBOW_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_FOOT_R", TW_TYPE_BOOL8, &ARS_LEFT_FOOT_R, "  group=reverseL   label='ARS_LEFT_FOOT_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_ANKLE_R", TW_TYPE_BOOL8, &ARS_LEFT_ANKLE_R, "  group=reverseL   label='ARS_LEFT_ANKLE_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_KNEE_R", TW_TYPE_BOOL8, &ARS_LEFT_KNEE_R, "  group=reverseL   label='ARS_LEFT_KNEE_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_THIGH_R", TW_TYPE_BOOL8, &ARS_LEFT_THIGH_R, " group=reverseL   label='ARS_LEFT_THIGH_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_UPPER_THIGH_R", TW_TYPE_BOOL8, &ARS_LEFT_UPPER_THIGH_R, "  group=reverseL   label='ARS_LEFT_UPPER_THIGH_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_LOWER_SHOULDER_R", TW_TYPE_BOOL8, &ARS_LEFT_LOWER_SHOULDER_R, " group=reverseL   label='ARS_LEFT_LOWER_SHOULDER_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_HIGH_SHOULDER_R", TW_TYPE_BOOL8, &ARS_LEFT_HIGH_SHOULDER_R, "  group=reverseL   label='ARS_LEFT_HIGH_SHOULDER_R'");
	TwAddVarRW(servoDefault, "ARS_LEFT_ELBOW_R", TW_TYPE_BOOL8, &ARS_LEFT_ELBOW_R, " group=reverseL   label='ARS_LEFT_ELBOW_R'");
}

void TW_CALL resetToStandPosition(void *clientData){
	defaultStandInstance(robot->robotCom);
}

void TW_CALL resetStepParameter(void *clientData){
	robot->b_parametricL = 1.0f;
	robot->b_parametricR = 0.0f;
}