//
//
//

#include "XPIDDCMotor.h"

namespace
{
}

XPIDDCMotor::XPIDDCMotor()
{
	setK(0);
	setP(0);
	setI(0);
	setD(0);
	target = 0;
	current = 0;
	standby = 0;
	last_error = 0;
	integrated_error = 0;
	power = 0;
	valmin = 0;
	valmax = 0;
	deadzone = 0;
}

void XPIDDCMotor::setK(double val) {
	K = val;
}

void XPIDDCMotor::setP(double val) {
	P = val;
}

void XPIDDCMotor::setI(double val){
	I = val;
}

void XPIDDCMotor::setD(double val) {
	D = val;
}

void XPIDDCMotor::setTarget(int val) {
	target = val;
}

void XPIDDCMotor::setCurrent(int val) {
	current = val;
}


void XPIDDCMotor::setStandby(int val) {
	standby = val;
}


void XPIDDCMotor::setDeadZone(int val) {
	deadzone = val;
}

void XPIDDCMotor::setMin(int val) {
	valmin = val;
}

void XPIDDCMotor::setMax(int val) {
	valmax = val;
}

/*void XPIDDCMotor::setPower(int val) {
	power = val;
}
*/

double XPIDDCMotor::getK() {
	return K;
}

double XPIDDCMotor::getP() {
	return P;
}

double XPIDDCMotor::getI() {
	return I;
}

double XPIDDCMotor::getD() {
	return D;
}

int XPIDDCMotor::getTarget() {
	return target;

}

int XPIDDCMotor::getCurrent() {
	return current;
}

int XPIDDCMotor::getStandby() {
	return standby;
}

int XPIDDCMotor::getPower() {
	return power;
}

int XPIDDCMotor::getDeadzone() {
	return deadzone;
}
int XPIDDCMotor::getMin() {
	return valmin;
}
int XPIDDCMotor::getMax() {
	return valmax;
}

int XPIDDCMotor::PIDUpdate() {
#define   GUARD_MOTOR_GAIN   100.0
	float error = (float)target - (float)current;
	float pTerm = P * error;
	integrated_error += error;
	float iTerm = I * constrain(integrated_error, -GUARD_MOTOR_GAIN, GUARD_MOTOR_GAIN);
	float dTerm = D * (error - last_error);
	last_error = error;
	
	if ( abs(target - current) <= (deadzone)  ) {
		power = 0;
	}
	else {
		power = int(constrain(K * (pTerm + iTerm + dTerm), -255, 255) / 2);;
	}
	return power;
}
