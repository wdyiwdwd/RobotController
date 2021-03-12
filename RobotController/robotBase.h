#include <memory>
#include "Aria.h"
#include "ArActionAvoidBump.h"	

using namespace std;

class RobotBase
{
private:
	std::shared_ptr<ArRobotConnector> connector;
	std::shared_ptr<ArArgumentParser> parser;
	std::shared_ptr<ArSonarDevice> sonarDev;
	std::shared_ptr<ArKeyHandler> keyHandler;
	std::shared_ptr<ArRobot> robot;

	//std::shared_ptr<ArActionLimiterForwards> forwardsLimiter;
	//std::shared_ptr<ArActionAvoidFront> frontAvoid;
	//std::shared_ptr<ArActionAvoidSide> sideAvoid;
	//std::shared_ptr<ArActionLimiterBackwards> backwardsLimiter;
	std::shared_ptr<ArActionAvoidBump> avoidAction;
	
	int stepLength;  //step length of forward
	int stepBack;  //step length of backward
	int stepRotation; //step size of rotation

	double pX; //postion x
	double pY; //postion y
	double pTh; //heading angle theta

	void initPostion();
public:
	RobotBase(int argc, char **argv);
	RobotBase();
	~RobotBase();
	void initParser(int argc, char **argv);
	bool connect();
	void attachSonar();
	void attachKeyBoard();
	void run();
	void setStepSize(int stepLength = 100, int stepBack = 100, int stepRotation = 10);
	void setMaxVel(int maxVel = 100);
	void setBumpDistance(int distance = 200);
	void updatePosition();
	void recordOdoDelInfo(char filename[]); //output delta information of Odometry then update position
	void moveFoward();
	void moveBackward();
	void rotate();
	void rotateLeft();
	void rotateRight();
	void move(double distance);
	void rotate(double angle);
	void stop();
	void waitForRunExit();
	void shutdown();
};