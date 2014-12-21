#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

class Joint;
class Structure;
class End;
class Member;

class Structure {
public:
	Structure();
	~Structure();
	void addJoint(Joint& joint);
	string to_string();
	void analyze(int method);
	void print();
private:
	vector<Joint> joints;
};

class End {
public:
	Joint& joint;
	double df;
	double cf;
	End(Joint& joint, double df, double cf, double m);

	double getMoment();
	void decrMoment(double moment);
private:
	double moment;
};

class Joint {
public:
	string name;
	Joint(string name);
	string to_string();
};

class Member {

};