#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <vector>

using namespace std;

const double TOLERANCE = 0.001;

class Joint;
class Structure;
class End;
class Member;

class Structure {
public:
	Structure();
	~Structure();
	void addJoint(Joint* joint);
	string to_string();
	void analyzeSequential();
	void analyzeParallel();
	void print();
	void makeMember(End* end1, End* end2);
private:
	vector<Joint*> joints;
};

class Joint {
public:
	string name;
	bool isFixed;
	Joint(string name, bool isFixed);
	void addEnd(End* end);
	bool release();
	string to_string();
private:
	vector<End*> ends;
};

class End {
public:
	string name;
	End(Joint* joint, double df, double cf, double m);
	void setFarEnd(End* farEnd);
	double getMoment();
	void distribute(double unbalancedMoment);
	string to_string();
private:
	Joint* joint;
	double df;
	double cf;
	End* farEnd;
	double moment;

	void decrMoment(double moment);
};