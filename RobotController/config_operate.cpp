#include "config_operate.h"  
#include <fstream>  
#include <iostream>  
using namespace std;

bool IsSpace(char c)//�ж��ǲ��ǿո�  
{
	if (' ' == c || '\t' == c)
		return true;
	return false;
}

bool IsCommentChar(char c)//�ж��ǲ���ע�ͷ�  
{
	switch (c) {
	case COMMENT_CHAR:
		return true;
	default:
		return false;
	}
}

void Trim(string & str)//ȥ���ַ�������β�ո�  
{
	if (str.empty()) {
		return;
	}
	int i, start_pos, end_pos;
	for (i = 0; i < str.size(); ++i) {
		if (!IsSpace(str[i])) {
			break;
		}
	}
	if (i == str.size()) { // ȫ���ǿհ��ַ���  
		str = "";
		return;
	}

	start_pos = i;

	for (i = str.size() - 1; i >= 0; --i) {
		if (!IsSpace(str[i])) {
			break;
		}
	}
	end_pos = i;

	str = str.substr(start_pos, end_pos - start_pos + 1);
}

//����name�������������  
//����ֵ����Ӧ������name��valueֵ  
string config_operate::getValue(const string & name)
{
	string line;
	string new_line;
	infile->seekg(0, ios::beg);
	while (getline(*infile, line))
	{
		if (line.empty())
		{
			continue;
			//return "Wrong";
		}
		int start_pos = 0, end_pos = line.size() - 1, pos;
		if ((pos = line.find(COMMENT_CHAR)) != -1)
		{
			if (0 == pos)
			{  // �еĵ�һ���ַ�����ע���ַ�  
				//return "";
				continue;
			}
			end_pos = pos - 1;
		}
		new_line = line.substr(start_pos, start_pos + 1 - end_pos);  // Ԥ����ɾ��ע�Ͳ���   
		if ((pos = new_line.find('=')) == -1)
		{
			return "";  // û��=��  
		}
		string na = new_line.substr(0, pos);
		Trim(na);
		if (na == name)
		{
			string value = new_line.substr(pos + 1, end_pos + 1 - (pos + 1));
			Trim(value);
			return  value;		
		}
	}
	return "";
}

config_operate::config_operate(const string & filename)
{
	infile = new ifstream(filename.c_str());
	if (!infile)
	{
		cout << "�޷��������ļ�" << endl;
	}
}

double config_operate::getNumber(const string & name)
{
	string value = this->getValue(name);
	return atof(value.c_str());
}

config_operate::config_operate(void)
{
}
config_operate::~config_operate(void)
{}