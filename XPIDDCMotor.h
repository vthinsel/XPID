// XPIDDCMotor.h

#ifndef _XPIDDCMOTOR_h
#define _XPIDDCMOTOR_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


#endif

class XPIDDCMotor
{
public:
	XPIDDCMotor();
	int PIDUpdate();
	void setK(double K);
	void setP(double P);
	void setI(double I);
	void setD(double D);
	void setTarget(int target);
	void setCurrent(int current);
	void setStandby(int standby);
	void setDeadZone(int val);
	void setMin(int val);
	void setMax(int val);
	//void setPower(int val);
	
	double getK();
	double getP();
	double getI();
	double getD();
	int getTarget();
	int getCurrent();
	int getStandby();
	int getPower();
	int getDeadzone();
	int getMin();
	int getMax();

private:
	double K;
	double P;
	double I;
	double D;
	int target;
	int current;
	int standby;
	int power;
	int valmax;
	int valmin;
	int deadzone;

	double integrated_error;
	float last_error;
	
};