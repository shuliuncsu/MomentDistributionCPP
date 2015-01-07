#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <atomic> 
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <vector>
#include <locale> 
#include <map>
#include <algorithm>

using namespace std;

const double TOLERANCE = 0.0001;

class Joint;
class Structure;
class End;
class Member;

class Structure {
public:
	Structure();
	~Structure();
	void add_joint(Joint* joint);
	string to_string();
	void analyze_sequential_Jacobi();
	void analyze_sequential_Gauss_Seidel();
	void analyze_parallel();
	void analyze_manual();
	void print();
	void make_member(End* end1, End* end2);
	double get_max_unbalanced_moment();
	bool finish = false;
private:
	map<string, Joint*> joints;
	void analyze_joint_thread(Joint* joint);
	void monitor();
	void analyze_schedule(vector<string> & schedule);
};

class Joint {
public:
	string name;
	bool is_fixed;
	Structure* s;

	mutex mtx;
	condition_variable cvj;

	Joint(string name, bool is_fixed, Structure* s);
	void add_end(End* end);
	bool release_par();
	bool release_schedule_1();
	bool release_schedule_2();
	double get_unbalanced_moment();
	string to_string();
private:
	vector<End*> ends;
	double unbalancedMoment = 0.0;
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