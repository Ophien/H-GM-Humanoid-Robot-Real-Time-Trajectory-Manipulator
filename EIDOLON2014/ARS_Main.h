#pragma once
#include <vector>
#include "ARSBehavior.h"

using namespace std;

class ARS_Main : public ARSBehavior
{
public:
	ARS_Main(void);
	~ARS_Main(void);

	void initialize();
	void preUpdate();
	void posUpdate();
	void update();
	void draw();
	void insertBehavior(ARSBehavior* behavior);

private:
	vector<ARSBehavior*> behaviors;
};

