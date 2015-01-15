#include "Structure.h"

//-------------Structure Class Functions---------------

//add a joint to the structure
void Structure::add_joint(Joint* joint) {
	joints.insert({ joint->name, joint });
}

//create a member, connect two ends
void Structure::make_member(End* end1, End* end2) {
	end1->set_far_end(end2);
	end2->set_far_end(end1);
}

//get the absolute value of the maximum unbalanced moment
double Structure::get_max_unbalanced_moment() {
	double max_unbalanced_moment = 0;
	for (auto pair : joints) {
		if (!pair.second->is_fixed)
			max_unbalanced_moment = max(max_unbalanced_moment, abs(pair.second->get_unbalanced_moment()));
	}
	return max_unbalanced_moment;
}

//apply Hardy Cross Moment Distribution Method on the structure using Jacobi schedule
void Structure::analyze_sequential_Jacobi() {
	int count = 0;
	vector<string> schedule;
	//add all joints to schedule
	for (auto pair : joints) {
		if (!pair.second->is_fixed)
			schedule.push_back(pair.first);
	}
	//perform iterations using same schedule
	while (get_max_unbalanced_moment() > TOLERANCE) {
		cout << endl << "Iteration " << ++count << ":" << endl;
		analyze_schedule(schedule);
	}
}

//apply Hardy Cross Moment Distribution Method on the structure using Gauss-Seidel schedule
void Structure::analyze_sequential_Gauss_Seidel() {
	int count = 0;
	vector<string> schedule;
	while (get_max_unbalanced_moment() > TOLERANCE) {
		for (auto pair : joints) {
			if (!pair.second->is_fixed && get_max_unbalanced_moment() > TOLERANCE) {
				schedule.clear();
				//add one joint to schedule
				schedule.push_back(pair.first);
				cout << endl << "Iteration " << ++count << ":" << endl;
				//perform iteration
				analyze_schedule(schedule);
			}
		}
	}
}

//apply Hardy Cross Moment Distribution Method on the structure using asynchronous parallel schedule
void Structure::analyze_parallel() {
	//connect joints as ring
	//initialize joint color and token for Dijkstra ring based termination algorithm
	Joint* temp = NULL;
	Joint* first = NULL;
	for (auto pair : joints) {
		if (!temp) {
			temp = first = pair.second;
			first->is_initializer = true;
			first->set_token(Token::black);
		}
		else {
			pair.second->next_joint = temp;
			pair.second->is_initializer = false;
			pair.second->set_token(Token::none);
			temp = pair.second;
		}
	}
	first->next_joint = temp;
	finish = false;

	//create thread pool
	vector<thread> pool;
	for (auto pair : joints) {
		Joint* joint = pair.second;
		pool.push_back(thread(&Structure::analyze_joint_thread_Dijkstra, this, joint));
	}

	//join threads
	for (auto& t : pool) {
		t.join();
	}
}

//apply Hardy Cross Moment Distribution Method on the structure using manual schedule
void Structure::analyze_manual() {
	int count = 0;
	string line;
	vector<string> schedule;

	do {
		schedule.clear();
		//prompt user input
		cout << endl << "Enter joints to be released [ ";
		for (auto pair : joints) {
			cout << pair.first << " ";
		}
		cout << "] (separate by comma, e.g. A,B) (0 to exit): ";
		cin >> line;

		if (line != "0") {
			for (size_t i = 0; i < line.length(); ++i)
				line[i] = toupper(line[i]);

			//prepare schedule based on user input
			int prev = 0;
			for (size_t i = 0; i <= line.size(); i++) {
				if (i == line.size() || line[i] == ',') {
					if (prev != i) {
						schedule.push_back(line.substr(prev, i - prev));
					}
					prev = i + 1;
				}
			}

			//perform analyze using user input schedule
			cout << endl << "Iteration " << ++count << ":" << endl;
			analyze_schedule(schedule);
		}
	} while (line != "0");
}

//apply one step of Hardy Cross Moment Distribution Method on the structure using a particular schedule
void Structure::analyze_schedule(vector<string> & schedule) {
	cout << "Releasing Joint:";
	for (auto s : schedule) {
		if (joints.find(s) != joints.end()) {
			//calculate unbalanced moment
			joints[s]->release_schedule_1();
			cout << " " << s;
		}
		else {
			cout << "Error: Joint " << s << " is not in structure." << endl;
		}
	}
	for (auto s : schedule) {
		if (joints.find(s) != joints.end()) {
			//distribute unbalanced moment
			joints[s]->release_schedule_2();
		}
	}
	print();
}

//print out structure's information
void Structure::print() {
	cout << endl << this->to_string() << endl;
	for (auto pair : joints) {
		Joint* joint = pair.second;
		cout << joint->to_string();
	}
}

//helper analysis function for a worker thread of a joint
void Structure::analyze_joint_thread_Dijkstra(Joint* joint) {
	while (!finish) {
		//repeat analysis until converge
		joint->release_parallel_Dijkstra();
	}
	cout << joint->name << " terminate" << endl;
	joint->next_joint->cvj.notify_all();
}

//get a string representation of the structure
string Structure::to_string() {
	return "Structure:\n\t# Nodes: " + std::to_string(joints.size())
		+ "\tMaximum Unbalanced Moment: " + std::to_string(get_max_unbalanced_moment());
}

//--------------Joint Class Functions---------------

//constructor, joint name is in uppercase
Joint::Joint(string name, bool is_fixed, Structure* s) : name(name), is_fixed(is_fixed), s(s) {
	for (size_t i = 0; i < name.length(); ++i)
		name[i] = toupper(name[i]);
}

//connect an end to a joint
void Joint::add_end(End* end) {
	ends.push_back(end);
}

//get the unbalanced moment of a joint
double Joint::get_unbalanced_moment() {
	double result = 0;
	for (End* end : ends) {
		result += end->get_moment();
	}
	return result;
}

//release a joint, change color and pass token based on Dijkstra ring based termination algorithm
//used in parallel analysis only
void Joint::release_parallel_Dijkstra() {
	unique_lock<mutex> lck(mtx);

	if (!release_schedule_1()) {
		//active thread
		release_schedule_2();
		for (End* end : ends) {
			//rule 1
			is_white = false;
			//notify other threads
			end->far_end->joint->cvj.notify_all();
		}
	}
	else {
		//passive thread
		while (!token_lock.try_lock()) {
			cvj.wait(lck);
		}
		if (is_initializer) {
			if (token == Token::black) {
				cout << "Joint " << name << " reveive black token, unsuccessful" << endl;
				//unsuccessful, initialize another white oken
				//rule 3, 4
				token = Token::none;
				is_white = true;
				cout << "Joint " << name << " sent white token" << endl;
				next_joint->set_token(Token::white);

				cvj.wait(lck);
			}
			else if (token == Token::white) {
				cout << "Joint " << name << " reveive white token, successful" << endl;
				//successful, terminate
				//rule 3
				s->finish = true;
				next_joint->cvj.notify_all();
			}
			else {
				cout << "No token at joint " << name << endl;

				cvj.wait(lck);
			}
		}
		else{
			//rule 0
			if (token != Token::none) {
				cout << "Joint " << name << " reveive " << (token == Token::white ? "white" : "black") << " token" << endl;
				//rule 2
				Token token_to_be_send = is_white ? token : Token::black;
				//rule 5
				token = Token::none;
				is_white = true;

				cout << "Joint " << name << " sent " << (token_to_be_send == Token::white ? "white" : "black") << " token" << endl;
				next_joint->set_token(token_to_be_send);
			}
			else {
				cout << "No token at joint " << name << endl;
			}

			cvj.wait(lck);
		}
		token_lock.unlock();
	}
}

//set token for a joint, then notify this joint
void Joint::set_token(Token token) {
	while (!token_lock.try_lock()) {
		cvj.notify_all();
	}
	this->token = token;
	cvj.notify_all();
	token_lock.unlock();
}

//prepare releasing a joint by calculating and recording its unbalanced moment 
bool Joint::release_schedule_1() {
	if (is_fixed) return true;
	unbalanced_moment = get_unbalanced_moment();
	return abs(unbalanced_moment) <= TOLERANCE;
}

//complete releasing a joint by redistributing its unbalanced moment 
void Joint::release_schedule_2() {
	if (!is_fixed && abs(unbalanced_moment) > TOLERANCE) {
		for (End* end : ends) {
			end->distribute(unbalanced_moment);
		}
	}
}

//get a string representation of the joint
string Joint::to_string() {
	string res = "\tJoint: " + name + (is_fixed ? " - Fixed " : "") + "\n";
	for (End* end : ends) {
		res += "\t\t" + end->to_string() + "\n";
	}
	return res;
}

//--------------End Class Functions---------------

//constructor
End::End(Joint* joint, double distribution_factor, double carryover_factor, double m)
	: joint(joint), name(joint->name), far_end(NULL), distribution_factor(distribution_factor), carryover_factor(carryover_factor), moment(m) {
	joint->add_end(this);
}

//connect the far end of the same member to this end
void End::set_far_end(End* far_end) {
	this->far_end = far_end;
	name = joint->name + far_end->joint->name;
}

//getter for the end moment
double End::get_moment() {
	return moment;
}

//distribute the unbalanced moment and calculate carryover moment
void End::distribute(double unbalanced_moment) {
	decr_moment(unbalanced_moment * distribution_factor); //mimic matrix D
	far_end->decr_moment(unbalanced_moment * distribution_factor * carryover_factor); //mimic matrix CD
}

//change the moment of this end, this calculation is thread safe
void End::decr_moment(double moment) {
	while (!lock.try_lock());
	this->moment -= moment;
	lock.unlock();
}

//get a string representation of the end
string End::to_string() {
	return "End: " + name + " Moment: " + std::to_string(moment);
}