#include "Structure.h"

//-------------Structure---------------
Structure::Structure() {

}

Structure::~Structure() {

}

string Structure::to_string() {
	return "Structure:\n\t# Nodes: " + std::to_string(joints.size())
		+ "\tMaximum Unbalanced Moment: " + std::to_string(get_max_unbalanced_moment());
}

void Structure::analyze_sequential_Jacobi() {
	int count = 0;
	vector<string> schedule;
	for (auto pair : joints) {
		if (!pair.second->is_fixed)
			schedule.push_back(pair.first);
	}
	while (get_max_unbalanced_moment() > TOLERANCE) {
		cout << endl << "Iteration " << ++count << ":" << endl;
		analyze_schedule(schedule);
	}
}

void Structure::analyze_sequential_Gauss_Seidel() {
	int count = 0;
	vector<string> schedule;
	while (get_max_unbalanced_moment() > TOLERANCE) {
		for (auto pair : joints) {
			if (!pair.second->is_fixed && get_max_unbalanced_moment() > TOLERANCE) {
				schedule.clear();
				schedule.push_back(pair.first);
				cout << endl << "Iteration " << ++count << ":" << endl;
				analyze_schedule(schedule);
			}
		}
	}
}

void Structure::analyze_parallel() {
	//unsigned int n = std::thread::hardware_concurrency();
	//std::cout << n << " concurrent threads are supported.\n";

	vector<thread> pool;
	for (auto pair : joints) {
		Joint* joint = pair.second;
		if (!joint->is_fixed)
			pool.push_back(thread(&Structure::analyze_joint_thread, this, joint));
	}

	thread master(&Structure::monitor, this);

	master.join();
	for (auto& t : pool) {
		t.join();
	}
}

void Structure::analyze_manual() {
	int count = 0;

	cout << endl << "Enter joints to be released [ ";
	for (auto pair : joints) {
		cout << pair.first << " ";
	}
	cout << "] (separate by comma, e.g. A,B): ";

	string line;
	vector<string> schedule;
	cin >> line;

	while (line != "0") {
		for (size_t i = 0; i < line.length(); ++i)
			line[i] = toupper(line[i]);

		int prev = 0;
		for (size_t i = 0; i <= line.size(); i++) {
			if (i == line.size() || line[i] == ',') {
				if (prev != i) {
					schedule.push_back(line.substr(prev, i - prev));
				}
				prev = i + 1;
			}
		}

		cout << endl << "Iteration " << ++count << ":" << endl;
		analyze_schedule(schedule);

		schedule.clear();
		cout << endl << "Enter joints to be released [ ";
		for (auto pair : joints) {
			cout << pair.first << " ";
		}
		cout << "] (separate by comma, e.g. A,B): ";
		cin >> line;
	}
}

void Structure::analyze_schedule(vector<string> & schedule) {
	cout << "Releasing Joint:";
	for (auto s : schedule) {
		if (joints.find(s) != joints.end()) {
			joints[s]->release_schedule_1();
			cout << " " << s;
		}
		else {
			cout << "Error: Joint " << s << " is not in structure." << endl;
		}
	}
	for (auto s : schedule) {
		if (joints.find(s) != joints.end()) {
			joints[s]->release_schedule_2();
		}
	}
	print();
}

double Structure::get_max_unbalanced_moment() {
	double max_unbalanced_moment = 0;
	for (auto pair : joints) {
		if (!pair.second->is_fixed)
			max_unbalanced_moment = max(max_unbalanced_moment, abs(pair.second->get_unbalanced_moment()));
	}
	return max_unbalanced_moment;
}

void Structure::monitor() {
	this_thread::sleep_for(std::chrono::seconds(1));
	finish = true;
	for (auto pair : joints) {
		Joint* joint = pair.second;
		joint->cvj.notify_all();
	}
}

void Structure::analyze_joint_thread(Joint* joint) {
	for (int i = 0; i < 1000 && !finish; ++i){
		joint->release_par();
		cout << joint->name;
		//this_thread::yield();
	}
}

void Structure::print() {
	cout << endl << this->to_string() << endl;
	for (auto pair : joints) {
		Joint* joint = pair.second;
		cout << joint->to_string();
	}
}

void Structure::add_joint(Joint* joint) {
	joints.insert({joint->name, joint});
}

void Structure::make_member(End* end1, End* end2) {
	end1->set_far_end(end2);
	end2->set_far_end(end1);
}

//--------------End-------------------
End::End(Joint* joint, double df, double cf, double m)
	: joint(joint), df(df), cf(cf), moment(m) {
	this->name = joint->name;
	far_end = NULL;
	joint->add_end(this);
}

void End::set_far_end(End* far_end) {
	this->far_end = far_end;
	this->name = joint->name + far_end->joint->name;
}

double End::get_moment() {
	return moment;
}

void End::distribute(double unbalancedMoment) {
	this->decr_moment(unbalancedMoment * df);
	this->far_end->decr_moment(unbalancedMoment * df * cf);
}

void End::decr_moment(double moment) {
	lock.lock();
	this->moment -= moment;
	lock.unlock();
}

string End::to_string() {
	return "End: " + name + " Moment: " + std::to_string(moment);
}

//--------------Joint------------------
Joint::Joint(string name, bool is_fixed, Structure* s) : name(name), is_fixed(is_fixed), s(s) {
	for (size_t i = 0; i < name.length(); ++i)
		name[i] = toupper(name[i]);
}

string Joint::to_string() {
	string res = "\tJoint: " + name + "\n";
	for (End* end : ends) {
		res += "\t\t" + end->to_string() + "\n";
	}
	return res;
}

void Joint::add_end(End* end) {
	ends.push_back(end);
}

bool Joint::release_par() {
	if (is_fixed) {
		return true;
	}
	double unbalancedMoment = 0.0;
	for (End* end : ends) {
		unbalancedMoment += end->get_moment();
	}
	if (abs(unbalancedMoment) > TOLERANCE) {
		for (End* end : ends) {
			end->distribute(unbalancedMoment);
			end->far_end->joint->cvj.notify_all();
		}
		return false;
	}
	else {
		unique_lock<mutex> lck(mtx);
		cvj.wait(lck);
		return true;
	}
}

bool Joint::release_schedule_1() {
	if (is_fixed) return true;
	unbalancedMoment = get_unbalanced_moment();
	return abs(unbalancedMoment) <= TOLERANCE;
}

bool Joint::release_schedule_2() {
	if (is_fixed) return true;
	if (abs(unbalancedMoment) > TOLERANCE) {
		for (End* end : ends) {
			end->distribute(unbalancedMoment);
		}
		return false;
	}
	else {
		return true;
	}
}

double Joint::get_unbalanced_moment() {
	double result = 0;
	for (End* end : ends) {
		result += end->get_moment();
	}
	return result;
}