#pragma once
#include "ariaTypedefs.h"
#include "ArAction.h"
#include "ArRobot.h"

#define ROBOT_NEAR 1000
#define MAX_DISTANCE 1000000

#define STATE_NORMAL 0
#define STATE_ADJUST 1
#define STATE_STOP 2

class ArActionAvoidBump :
	public ArAction
{
private:
	int state;
	int bumpDistance;
	int backStep;
	int frontStep;
	int rotationStep;
	int turnStep;
public:
	ArActionAvoidBump(const char *name);
	~ArActionAvoidBump();
	void setBumpDistance(int distance);
	ArActionDesired* fire(ArActionDesired currentDessired);
};

