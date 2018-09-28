#include "ARS_Main.h"

ARS_Main::ARS_Main(void){
}

ARS_Main::~ARS_Main(void){
}

void ARS_Main::initialize(){
	for(unsigned int i = 0; i < behaviors.size(); i++)
				if(behaviors[i]->enable)
		behaviors[i]->initialize();
}

void ARS_Main::preUpdate(){}
void ARS_Main::posUpdate(){}

void ARS_Main::update(){
	for(unsigned int i = 0; i < behaviors.size(); i++)
		if(behaviors[i]->enable)
		behaviors[i]->preUpdate();

	for(unsigned int i = 0; i < behaviors.size(); i++)
				if(behaviors[i]->enable)
		behaviors[i]->update();

	for(unsigned int i = 0; i < behaviors.size(); i++)
				if(behaviors[i]->enable)
		behaviors[i]->posUpdate();
}

void ARS_Main::insertBehavior(ARSBehavior* behavior){
	behaviors.push_back(behavior);
}

void ARS_Main::draw(){
	for(unsigned int i = 0; i < behaviors.size(); i++)
				if(behaviors[i]->enable)
		behaviors[i]->draw();
}