#include "stdafx.h"
#include "ArActionAvoidBump.h"


ArActionAvoidBump::ArActionAvoidBump(const char *name = "Avoid Bump") : ArAction(name, "Avoid Bumping")
{
	this->state = STATE_NORMAL;
}


ArActionAvoidBump::~ArActionAvoidBump()
{
	this->bumpDistance = 300;
	this->frontStep = bumpDistance / 5;
	this->backStep = -bumpDistance / 5;
	this->turnStep = frontStep * 0.7;
	this->rotationStep = 30;
}

void ArActionAvoidBump::setBumpDistance(int distance) {
	this->bumpDistance = distance;
	this->frontStep = distance / 5;
	this->backStep = -distance / 5;
	this->turnStep = frontStep * 0.7;
	this->rotationStep = 30;
}

ArActionDesired* ArActionAvoidBump::fire(ArActionDesired currentDesired)
{
	double leftDist, rightDist, frontDist, backDist;
	leftDist = rightDist = frontDist = backDist = MAX_DISTANCE;
	double leftAngle, rightAngle, frontAngle, backAngle = 0;
	ArSensorReading* sonarReading;
	for (int i = 0; i < myRobot->getNumSonar(); i++) {
		sonarReading = myRobot->getSonarReading(i);
		double theta = sonarReading->getSensorTh();
		double dist = sonarReading->getRange();
		//printf("num = %d, theta = %f, dist = %f\n", i, theta, dist);
		if (theta <= 30 && theta >= -30) {
			//在机器人前方
			if (frontDist > sonarReading->getRange()){
				frontDist = dist;
				frontAngle = theta;
			}
		}
		else if (theta >= 150 || theta <= -150) {
			//在机器人后方
			if (backDist > sonarReading->getRange()){
				backDist = dist;
				backAngle = theta;
			}
		}
		else if (theta > 30 && theta < 150) {
			//在机器人左侧
			if (leftDist > sonarReading->getRange()){
				leftDist = dist;
				leftAngle = theta;
			}
		}
		else if (theta < -30 && theta > -150) {
			//在机器人右侧
			if (rightDist > sonarReading->getRange()){
				rightDist = dist;
				rightAngle = theta;
			}
		}
	}
	//对机器人状态进行调整离开与障碍物太近的区域
	if (state == STATE_ADJUST){
		if (frontDist <= this->bumpDistance) {
			myRobot->setVel(0);
			myRobot->move(this->backStep);
			state = STATE_NORMAL;
		}
		if (backDist <= this->bumpDistance) {
			myRobot->setVel(0);
			myRobot->move(this->frontStep);
			state = STATE_NORMAL;
		}
		if (leftDist <= this->bumpDistance) {
			myRobot->setVel(0);
			myRobot->setDeltaHeading(-this->rotationStep);
			myRobot->move(this->turnStep);
			state = STATE_NORMAL;
		}
		if (rightDist <= this->bumpDistance) {
			myRobot->setVel(0);
			myRobot->setDeltaHeading(this->rotationStep);
			myRobot->move(this->turnStep);
			state = STATE_NORMAL;
		}
	}
	//机器人在正常状态下判定与障碍物的远近
	if (state == STATE_NORMAL){
		if (frontDist < ROBOT_NEAR) {
			if (frontDist <= this->bumpDistance) {
				state = STATE_ADJUST;
			}
			/*else{
				myRobot->setVel(0);
			}*/
		}
		if (backDist < ROBOT_NEAR) {
			if (backDist <= this->bumpDistance) {
				state = STATE_ADJUST;
			}
			/*else{
				myRobot->setVel(0);
			}*/
		}
		if (leftDist < ROBOT_NEAR) {
			if (leftDist <= this->bumpDistance) {
				state = STATE_ADJUST;
			}
		}
		if (rightDist < ROBOT_NEAR) {
			if (rightDist <= this->bumpDistance) {
				state = STATE_ADJUST;
			}
		}
	}
	return (&currentDesired);
}