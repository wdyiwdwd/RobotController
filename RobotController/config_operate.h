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

	//��ʼ�������ļ�������ļ����
	config_operate(const std::string& filename);

	//�Ӳ����ļ��У���ȡ���Ϊname��ֵ
	std::string getValue(const std::string& name);
	double getNumber(const std::string & name);
};


#endif;