//#include "config_operate.h"
//#include "utils.h"
//#include <iostream>
//
//using namespace std;
//
//int main()
//{
//	cout << "first started" << endl;
//
//	utils MyAPP;
//	
//	MyAPP.equipInitial();
//
//}

#include "robotBase.h"

int main(int argc, char** argv)
{
	RobotBase base(argc, argv);
	base.connect();
	base.attachSonar();
	base.run();
	base.moveFoward(3000);
	base.waitForRunExit();
	base.shutdown();
	return 0;
}