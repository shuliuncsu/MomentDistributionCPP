#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <locale> 
#include <map>
#include <algorithm>
#include <cmath>

using namespace std;

//tolerance of the maximum unbalanced moment
const double TOLERANCE = 0.0001;

class Joint;
class Structure;
class End;
class Member;

class Structure {
public:
	Structure() {};
	~Structure() {};

	void add_joint(Joint* joint);
	void make_member(End* end1, End* end2);
	double get_max_unbalanced_moment();

	void analyze_sequential_Jacobi();
	void analyze_sequential_Gauss_Seidel();
	void analyze_parallel();
	void analyze_manual();
	void analyze_schedule(vector<string> & schedule);

	void print();
	
	bool finish = false;
private:
	void analyze_joint_thread(Joint* joint);
	void monitor();
	string to_string();

	map<string, Joint*> joints;
};

class Joint {
public:
	Joint(string name, bool is_fixed, Structure* s);
	void add_end(End* end);

	double get_unbalanced_moment();
	bool release_par();
	bool release_schedule_1();
	bool release_schedule_2();
	
	string to_string();

	string name;
	bool is_fixed;
	condition_variable cvj;
private:
	Structure* s;
	vector<End*> ends;
	double unbalancedMoment = 0.0;
	mutex mtx;
};

class End {
public:
	string name;
	mutex lock;
	End(Joint* joint, double df, double cf, double m);
	void set_far_end(End* far_end);
	double get_moment();
	void distribute(double unbalanced_moment);
	string to_string();
	End* far_end;
	Joint* joint;
private:
	double df;
	double cf;
	double moment;

	void decr_moment(double moment);
};
