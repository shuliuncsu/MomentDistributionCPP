#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include "Structure.h"

using namespace std;

int main(int argc, char** argv) {
	Structure s = Structure();
	s.addJoint(Joint("A"));
	s.print();
	s.addJoint(Joint("B"));
	s.print();

	string temp;
	cin >> temp;
	return 0;
}

