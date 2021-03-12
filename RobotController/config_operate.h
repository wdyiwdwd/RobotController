/*************************************************
*  author: Glen Zhang
*  date  : 2017.6.21
*  to    : read config file 
*************************************************/

#ifndef CONFIG_OPERATE
#define CONFIG_OPERATE

#include <string>
#include <iostream>
#define COMMENT_CHAR '#'

class config_operate
{
private:
	std::ifstream *infile;
public:
	config_operate();
	~config_operate();

	//初始化参数文件，获得文件句柄
	config_operate(const std::string& filename);

	//从参数文件中，获取标记为name的值
	std::string getValue(const std::string& name);
	double getNumber(const std::string & name);
};


#endif;