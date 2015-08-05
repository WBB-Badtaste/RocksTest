#pragma once

#include "NyceExErrorHandle.h"
#include <iostream>
#include <fstream>

using namespace std;

void HandleError(NYCE_STATUS Status, const char *name)
{
	cout<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusStringEx(Status)<<"\n"<<endl;
	ofstream file("..//ErrorLog.txt");	
	file<<"\n\nError occur at:"<<name<<"\nError Code:"<<NyceGetStatusStringEx(Status)<<"\n"<<endl;
	file.close();
}