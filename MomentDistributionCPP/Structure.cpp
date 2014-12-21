#include "Structure.h"

Structure::Structure() {

}

Structure::~Structure() {

}

string Structure::to_string() {
	return "Structure:\n# Nodes: " + std::to_string(joints.size());
}

void Structure::analyze(int method) {
	/*	thread t1(&Structure::print, this, 1);
		thread t2(&Structure::print, this, 2);
		t1.join();
		t2.join();*/
}

void Structure::print() {
	cout << this->to_string() << endl;
}

void Structure::addJoint(Joint& joint) {
	joints.push_back(joint);
}

Joint::Joint(string name) {
	this->name = name;
}

string Joint::to_string() {
	return "Joint " + name;
}