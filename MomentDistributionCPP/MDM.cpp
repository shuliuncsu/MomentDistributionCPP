#include "MDM.h"

//Hardy Cross Method of Moment Distribution
int main(int argc, char** argv) {
	cout << "*************************************************" << endl;
	cout << "*   Hardy Cross Method of Moment Distribution   *" << endl;
	cout << "*************************************************" << endl;

	//Repeatly prompt user to choose actions
	while (true) {
		//choose which example problem to be analyzied
		cout  << endl << "Please enter problem index (1 or 2) (0 to exit): ";
		int index;
		auto problem = problem1;
		cin >> index;

		if (index <= 0) return 0;

		switch (index) {
		case 1:
			problem = problem1;
			break;
		case 2:
			problem = problem2;
			break;
		}

		//print structure before analysis
		Structure* s;
		s = problem();
		s->print();

		//choose analysis type
		cout << endl <<  "Please enter analysis type (1: sequential - Jacobi, 2: sequential - Gauss-Seidel, 3: parallel, 4: manual): ";
		int type;
		cin >> type;
		switch (type) {
		case 1:
			s->analyze_sequential_Jacobi();
			break;
		case 2:
			s->analyze_sequential_Gauss_Seidel();
			break;
		case 3:
			s->analyze_parallel();
			s->print();
			break;
		case 4:
			s->analyze_manual();
			break;
		}

	}
	return 0;
}

//example problem 1
Structure* problem1() {
	Structure* s = new Structure();

	Joint* jointA = new Joint("A", true, s);
	Joint* jointB = new Joint("B", false, s);
	Joint* jointC = new Joint("C", false, s);
	End* end1 = new End(jointA, 0.0, 0.5, -172.8);
	End* end2 = new End(jointB, 0.5, 0.5, 115.2);
	End* end3 = new End(jointB, 0.5, 0.5, -416.7);
	End* end4 = new End(jointC, 1, 0.5, 416.7);

	s->add_joint(jointA);
	s->add_joint(jointB);
	s->add_joint(jointC);

	s->make_member(end1, end2);
	s->make_member(end3, end4);

	return s;
}

//example problem 2
Structure* problem2() {
	Structure* s = new Structure();

	Joint* jointA = new Joint("A", false, s);
	Joint* jointB = new Joint("B", false, s);
	Joint* jointC = new Joint("C", false, s);
	Joint* jointD = new Joint("D", false, s);
	Joint* jointE = new Joint("E", true, s);
	Joint* jointF = new Joint("F", true, s);

	End* end1 = new End(jointA, 1, 0.5, 0);
	End* end2 = new End(jointB, 1.0 / 3.5, 0.5, 0);
	End* end3 = new End(jointB, 1.0 / 3.5, 0.5, -44.4);
	End* end4 = new End(jointC, 1.0 / 3.5, 0.5, 22.2);
	End* end5 = new End(jointC, 1.0 / 3.5, 0.5, -90);
	End* end6 = new End(jointD, 1, 0.5, 90);
	End* end7 = new End(jointB, 1.5 /3.5, 0.5, -25);
	End* end8 = new End(jointE, 1, 0.5, 25);
	End* end9 = new End(jointC, 1.5 / 3.5, 0.5, 0);
	End* end10 = new End(jointF, 1, 0.5, 0);

	s->add_joint(jointA);
	s->add_joint(jointB);
	s->add_joint(jointC);
	s->add_joint(jointD);
	s->add_joint(jointE);
	s->add_joint(jointF);

	s->make_member(end1, end2);
	s->make_member(end3, end4);
	s->make_member(end5, end6);
	s->make_member(end7, end8);
	s->make_member(end9, end10);

	return s;
}
