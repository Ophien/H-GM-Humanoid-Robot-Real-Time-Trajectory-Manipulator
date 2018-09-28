#pragma once
class ARSBehavior
{
public:
	ARSBehavior(void);
	~ARSBehavior(void);

	bool enable;

	virtual void initialize()=0;
	virtual void preUpdate() =0;
	virtual void update()    =0;
	virtual void posUpdate() =0;
	virtual void draw()      =0;
};

