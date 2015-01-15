#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
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

enum class Token {none, white, black};

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
	string to_string();
	
	bool finish = false;
private:
	void analyze_joint_thread_Dijkstra(Joint* joint);

	map<string, Joint*> joints;
};

class Joint {
public:
	Joint(string name, bool is_fixed, Structure* s);
	void add_end(End* end);
	double get_unbalanced_moment();
	bool release_schedule_1();
	void release_schedule_2();
	void release_parallel_Dijkstra();
	void set_token(Token token);
	string to_string();

	string name;
	bool is_fixed;
	condition_variable cvj;
	bool is_initializer = false;
	Joint* next_joint;
	bool is_white = false;
	mutex token_lock;
private:
	Structure* s;
	vector<End*> ends;
	double unbalanced_moment = 0.0;
	mutex mtx;
	Token token = Token::none;
};

class End {
public:
	End(Joint* joint, double df, double cf, double m);
	void set_far_end(End* far_end);
	double get_moment();
	void distribute(double unbalanced_moment);
	string to_string();

	string name;
	mutex lock;
	End* far_end;
	Joint* joint;
private:
	void decr_moment(double moment);

	double distribution_factor;
	double carryover_factor;
	double moment;
};
