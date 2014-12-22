#include "MDM.h"

int main(int argc, char** argv) {
	Structure s = problem1();

	s.print();

	s.analyzeSequential();

	s.print();

	string temp;
	cin >> temp;
	return 0;
}

Structure problem1() {
	Structure s = Structure();

	Joint* jointA = new Joint("A", true);
	Joint* jointB = new Joint("B", false);
	Joint* jointC = new Joint("C", false);
	End* end1 = new End(jointA, 0.0, 0.5, -172.8);
	End* end2 = new End(jointB, 0.5, 0.5, 115.2);
	End* end3 = new End(jointB, 0.5, 0.5, -416.7);
	End* end4 = new End(jointC, 1, 0.5, 416.7);

	s.addJoint(jointA);
	s.addJoint(jointB);
	s.addJoint(jointC);

	s.makeMember(end1, end2);
	s.makeMember(end3, end4);

	return s;
}