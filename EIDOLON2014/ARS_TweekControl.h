
#pragma once

#include "ARSRobotController.h"
#include "AntTweakBar.h"

extern	ARSRobotController* robot;

void createAllBars();

void TW_CALL resetToStandPosition(void *clientData);
void TW_CALL resetStepParameter(void *clientData);