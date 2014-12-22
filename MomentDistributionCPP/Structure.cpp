#include "Structure.h"

//-------------Structure---------------
Structure::Structure() {

}

Structure::~Structure() {

}

string Structure::to_string() {
	return "Structure:\n# Nodes: " + std::to_string(joints.size());
}

void Structure::analyzeSequential() {
	bool done;

	do {
		done = true;
		for (Joint* joint : joints) {
			done = joint->release() && done;
		}
	} while (!done);
}

void Structure::analyzeParallel() {
	/*	thread t1(&Structure::print, this, 1);
	thread t2(&Structure::print, this, 2);
	t1.join();
	t2.join();*/
}

void Structure::print() {
	cout << this->to_string() << endl;
	for (Joint* joint : joints) {
		cout << joint->to_string();
	}
}

void Structure::addJoint(Joint* joint) {
	joints.push_back(joint);
}

void Structure::makeMember(End* end1, End* end2) {
	end1->setFarEnd(end2);
	end2->setFarEnd(end1);
}

//--------------End-------------------
End::End(Joint* joint, double df, double cf, double m)
	: joint(joint), df(df), cf(cf), moment(m) {
	this->name = joint->name;
	farEnd = NULL;
	joint->addEnd(this);
}

void End::setFarEnd(End* farEnd) {
	this->farEnd = farEnd;
	this->name = joint->name + farEnd->joint->name;
}

double End::getMoment() {
	return moment;
}

void End::distribute(double unbalancedMoment) {
	this->decrMoment(unbalancedMoment * df);
	this->farEnd->decrMoment(unbalancedMoment * df * cf);
}

void End::decrMoment(double moment) {
	this->moment -= moment;
}

string End::to_string() {
	return "End: " + name + " Moment: " + std::to_string(moment);
}

//--------------Joint------------------
Joint::Joint(string name, bool isFixed) : name(name), isFixed(isFixed) {

}

string Joint::to_string() {
	string res = "\tJoint: " + name + "\n";
	for (End* end : ends) {
		res += "\t\t" + end->to_string() + "\n";
	}
	return res;
}

void Joint::addEnd(End* end) {
	ends.push_back(end);
}

bool Joint::release() {
	if (isFixed) return true;
	double unbalancedMoment = 0.0;
	for (End* end : ends) {
		unbalancedMoment += end->getMoment();
	}
	if (abs(unbalancedMoment) > TOLERANCE) {
		for (End* end : ends) {
			end->distribute(unbalancedMoment);
		}
		return false;
	} else {
		return true;
	}
}