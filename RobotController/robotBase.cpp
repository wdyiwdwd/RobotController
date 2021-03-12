#include "robotBase.h"

using namespace std;

RobotBase::RobotBase() {
	Aria::init();
	this->robot = std::make_shared<ArRobot>();
	//init the parameter of robot
	this->setStepSize();
	this->initPostion();
}

void RobotBase::initParser(int argc, char **argv) {
	this->parser = std::make_shared<ArArgumentParser>(&argc, argv);
	parser->loadDefaultArguments();
}

RobotBase::RobotBase(int argc, char **argv) {
	Aria::init();
	this->parser = std::make_shared<ArArgumentParser>(&argc, argv);
	parser->loadDefaultArguments();
	this->robot = std::make_shared<ArRobot>();
	//init the parameter of robot
	this->setStepSize();
	this->initPostion();
}

RobotBase::~RobotBase() {}

void RobotBase::initPostion() {
	this->pX = 0; 
	this->pY = 0; 
	this->pTh = 0;
}

void RobotBase::updatePosition() {
	this->pX = this->robot->getX();
	this->pY = this->robot->getY();
	this->pTh = this->robot->getTh();
}

void RobotBase::setStepSize(int stepLength , int stepBack, int stepRotation) {
	this->stepLength = stepLength;
	this->stepBack = stepBack;
	this->stepRotation = stepRotation;
}

void RobotBase::setMaxVel(int maxVel) {
	this->robot->lock();
	this->robot->setTransVelMax(maxVel);
	this->robot->unlock();
}

void RobotBase::setBumpDistance(int distance) {
	this->avoidAction->setBumpDistance(distance);
}

bool RobotBase::connect() {
	this->connector = std::make_shared<ArRobotConnector>(&(*this->parser), &(*this->robot));
	if (!this->connector->parseArgs()) {
		connector->logOptions();
		return 0;
	}
	if (!this->connector->connectRobot(&(*(this->robot)))) {
		printf("counld not connect to robot...exiting\n");
		Aria::shutdown();
		return 0;
	}
	return 1;
}

void RobotBase::attachSonar() {
	this->sonarDev = std::make_shared<ArSonarDevice>();
	this->robot->addRangeDevice(&(*(this->sonarDev)));
}

void RobotBase::attachKeyBoard() {
	this->keyHandler = std::make_shared<ArKeyHandler>(); 
	Aria::setKeyHandler(&(*(this->keyHandler)));
	this->robot->attachKeyHandler(&(*(this->keyHandler)));
}

void RobotBase::run() {
	//this->forwardsLimiter = std::make_shared<ArActionLimiterForwards>("limiter near", 300, 600, 100, 1);
	//this->backwardsLimiter = std::make_shared<ArActionLimiterBackwards>("limiter backwards", 200, 500, 100, 1);
	//this->frontAvoid = std::make_shared<ArActionAvoidFront>("front avoid", 1000, 1 , 90, true);
	//this->sideAvoid = std::make_shared<ArActionAvoidSide>("side avoid", 500, 50);
	this->robot->setRotVelMax(50);
	this->robot->runAsync(true);
	this->avoidAction = std::make_shared<ArActionAvoidBump>("Avoid Bumping");
	this->avoidAction->setRobot(&(*this->robot));
	this->robot->comInt(ArCommands::ENABLE, 1);
	this->robot->addAction(&(*this->avoidAction), 100);
}

void RobotBase::move(double distance) {
	//this->robot->setVel(500);
	//this->robot->setRotVel(10);
	this->robot->move(distance);
}

void RobotBase::rotate(double angle) {
	this->robot->setDeltaHeading(angle);
}


void RobotBase::moveFoward() {
	//this->robot->setVel(500);
	//this->robot->setRotVel(10);
	this->move(this->stepLength);
}

void RobotBase::moveBackward() {
	this->move(-this->stepBack);
}

void RobotBase::rotateLeft() {
	this->rotate(this->stepRotation);
}

void RobotBase::rotateRight() {
	this->rotate(-this->stepRotation);
}

void RobotBase::stop() {
	this->robot->stop();
}

void RobotBase::waitForRunExit() {
	this->robot->waitForRunExit();
}

void RobotBase::shutdown() {
	Aria::shutdown();
}
void RobotBase::recordOdoDelInfo(char filename[]) {
	//相对上一时刻的相对定位
	//double deltaX = this->robot->getX() - this->pX;
	//double deltaY = this->robot->getY() - this->pY;
	//double deltaTh = this->robot->getTh() - this->pTh;

	//相对开机时坐标原点的绝对定位
	double deltaX = this->robot->getX();
	double deltaY = this->robot->getY();
	double deltaTh = this->robot->getTh();
	FILE *fp = fopen(filename, "w");
	if ((fp == NULL))
	{
		exit(0);
	}
	else
	{
		fprintf(fp, "%15f", deltaX);
		fprintf(fp, "%15f", deltaY);
		fprintf(fp, "%15f", deltaTh);
		fprintf(fp, "\r\n");
		//Sleep(20);
	}
	fclose(fp);
	this->updatePosition();
}