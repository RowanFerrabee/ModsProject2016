/*
	When defining cantilever, define all joints, then define members between joints
	Initial test: calculate forces in members per unit P and see if correct
	Secondary test: output required widths and see if correct
	Run thru gradient descent and watch PV as is goes so we dont waste time on bad descent constants
*/

#include "Truss.h"
#include "matrix.h"
#include <cmath>
#include <fstream>

using namespace std;

void populateForces(vector<Member*> members, vector<Joint*> joints);
void populateTrussFromFile(string fileName, 
	vector<Joint*>& jointList,
	vector<Joint*>& moveableJointList, 
	vector<Member*>& members,
	double applied_force);
void outputMember(ostream &out, Member m);
double massOfMember(Member m);

void cleanup(vector<Joint*> &joints,
	vector<Joint*> &movableJoints,
	vector<Member*> &members) {

	while(movableJoints.size()) {
		Joint* j = movableJoints.back();
		movableJoints.pop_back();
		joints.pop_back();
		delete j;
	}
	while(joints.size()) {
		Joint* j = joints.back();
		joints.pop_back();
		delete j;
	}
	while(members.size()) {
		Member* m = members.back();
		members.pop_back();
		delete m;
	}
}

int main() {
	// string inFileName, outFileName;
	// cout << "Input file name: ";
	// cin >> inFileName;
	// cout << "Output file name: ";
	// cin >> outFileName;	

	string inFileName = "o.txt";
	string outFileName = "o_details_1.txt";

	vector<Joint*> joints;
	vector<Joint*> movableJoints;
	vector<Member*> members;

	// double maxPV = 0, maxAppliedForce;
	// for(double applied_force = 0.1; applied_force < 10.05; applied_force+=0.1) {
	// 	cout << "Iteration start: " << applied_force << "\n";
	// 	members.size();
	// 	populateTrussFromFile(inFileName, joints, movableJoints, members, applied_force);
	// 	cout << "File Read\n"
	// 		 << members.size() << " " << joints.size() << endl;

	// 	populateForces(members, joints);
	// 	cout << "Forces calculated\n";
	// 	double totalMass = 0;
	// 	for(int i = 0; i < members.size(); i++) 
	// 		totalMass += massOfMember(*(members.at(i)));
	// 	cout << "Sum calculated\n";
	// 	double PV = applied_force / totalMass; 
	// 	if(PV > maxPV) {
	// 		maxPV = PV;
	// 		maxAppliedForce = applied_force;
	// 	}
	// 	cout << "PV calculated\n";
	// 	cleanup(joints, movableJoints, members);
	// 	cout << "Iteration complete: " << applied_force << "\n";
	// }

	double maxAppliedForce = 60;

	populateTrussFromFile(inFileName, joints, movableJoints, members, maxAppliedForce);
	populateForces(members, joints);

	double totalMass = 0;
		for(int i = 0; i < members.size(); i++) 
			totalMass += massOfMember(*(members.at(i)));
	double maxPV = (maxAppliedForce / 9.81) * 1000  / totalMass;

	ofstream fout(outFileName.c_str());
	fout << "Truss Details \n"
	<< "Force applied: " << maxAppliedForce << "\n"
	<< "PV: " << maxPV << "\n"
	<< "Total Mass: " << totalMass << "\n\n";

	for(int i = 0; i < members.size(); i++) {
		outputMember(fout, *(members.at(i)));
	}

	/*


	vector<Joint*> joints;
	vector<Joint*> movableJoints;
	vector<Member*> members;
	double applied_force = 2

	string fileName;
	cout << "Input file name: ";
	cin >> fileName;
	populateTrussFromFile(fileName, joints, movableJoints, members, applied_force);
	cout << "Output file name: ";
	cin >> fileName;

	populateForces(members, joints);
	ofstream fout(fileName.c_str());

	fout << "Truss Details - one half of truss \n"
		 << "Applied force: " << applied_force << "N \n\n";

	double sum = 0;
	for(int i = 0; i < members.size(); i++) {
		sum += massOfMember(*(members.at(i)));
	}
	double PV = (applied_force / 9.81) * 1000  / sum;

	fout << "PV: " << PV << "\n\n";
	for(int i = 0; i < members.size(); i++) {
		outputMember(fout, *(members.at(i)));
	}
	*/
}

void populateForces(vector<Member*> memberList, vector<Joint*> joints) {
	Matrix forceMatrix(joints.size() * 2, memberList.size() + 1);
	//Be sure that the members vector is sorted by id

	if(joints.size()*2 < memberList.size() + 1) {
		cout << "Bridge makes no sense, force calculations make no sense." << endl;
		return;
	}

	for (int i=0; i<joints.size(); i++) {
		Joint* curJoint = joints.at(i);

		for (int j=0; j<curJoint->numMembers(); j++) {
			Joint* otherJoint = curJoint->members[j]->leftJoint() == curJoint ? curJoint->members[j]->rightJoint() : curJoint->members[j]->leftJoint();
			for (int k=0; k<joints.size(); k++) {
				//if (otherJoint == joints[k])
					//cout << "OtherJoint " << k << ": (" << otherJoint->getX() << "," << otherJoint->getY() << ")" << endl;
			}
			double angle = atan2(otherJoint->getY() - curJoint->getY(), otherJoint->getX() - curJoint->getX());
			//cout << angle << endl;
			forceMatrix.setElement(2*i, curJoint->members[j]->id(), cos(angle));
			forceMatrix.setElement(2*i+1, curJoint->members[j]->id(), sin(angle));
			//forceMatrix.print(cout);
		}
		forceMatrix.setElement(2*i, memberList.size(), -1 * curJoint->appliedX());
		forceMatrix.setElement(2*i+1, memberList.size(), -1 * curJoint->appliedY());
	}
	//forceMatrix.print(cout);
	forceMatrix = forceMatrix.rref();
	//forceMatrix.print(cout);
	for (int i=0; i<memberList.size(); i++) {
		memberList.at(i)->force = forceMatrix.getElement(i, memberList.size());
	}
}

void populateTrussFromFile(string fileName, 
	vector<Joint*>& jointList,
	vector<Joint*>& moveableJointList, 
	vector<Member*>& members,
	double applied_force) {

	jointList.push_back(new Joint(0,0.05));
	jointList.push_back(new Joint(0,0));
	jointList.push_back(new Joint(0.3,0));

	jointList[0]->setAppliedForce(-6 * applied_force, applied_force);
	jointList[1]->setAppliedForce(6 * applied_force, 0);
	jointList[2]->setAppliedForce(0, -1 * applied_force);

	ifstream fin(fileName.c_str());

	int n;
	double x, y; 
	fin >> n; //here, n equals number of joints
//	cout << "Number of joints: " << n <<  endl;
	for(int i = 0; i < n; i++) {
		fin >> x >> y; //here (x, y) is a coordinate
		Joint* j = new Joint(x, y);
		jointList.push_back(j);
		moveableJointList.push_back(j);
	}
	int l, r;
	fin >> n; // here, n equals number of members
//	cout << "Number of members: " << n << endl;
	for(int i = 0; i < n; i++) {
		fin >> l >> r; //here l and r are joint indexes
//		cout << l << " " << r << endl;
		if (l >= 0 && r >= 0 && l < jointList.size() && r < jointList.size()) {
			Member* newMember = new Member(jointList[l], jointList[r]);
			members.push_back(newMember);
			jointList[l]->members.push_back(newMember);
			jointList[r]->members.push_back(newMember);
		}
	}
//	cout << "Finished inputing member" << endl;
	fin.close();
}

void setMemberProperties(
	Member m,
	double &length, double &width,
	double &D, double &d,
	double &b,
	double &area
	) {
	const double allowableNormalStress = 14.6 * 1000000 / 2;	// stress(Pa) = stress(MPa)*1000000(Pa/MPa)/2
	const double allowableShearStress = 2.05 * 1000000 / 2;		// stress(Pa) = stress(MPa)*1000000(Pa/MPa)/2

	length = m.length();
	d = 0.003175;

	if(m.force > 0) {
		width = max(m.force / (d * allowableNormalStress), 0.003175);
		D = width + d;
		b = m.force / (2*allowableShearStress*d) + (d-D) / 2;
		area = 	2* d * (b + 0.5 * sqrt(2.25*d*d + D*D)) +
			M_PI / 2 * (D*D - d*d) +
			width * (length - sqrt(D*D - width*width));
	} else {
		width = max(3*d, -1 * 2 * m.force / (d * allowableNormalStress));
		D = 3*d;
		b = 0;
		area = (M_PI / 4) * (D*D - 2*d*d) + length * width;
	}
}

//output properties of member to file
void outputMember(ostream &out, Member m) {

	double length, width, D, d, b, areaOfTopFace;
	setMemberProperties(m, length, width, D, d, b, areaOfTopFace);
	double mass = areaOfTopFace * 0.003175 * 0.141 * 1000000;		//mass (g) = area(m^2)*depth(m)*density(g/m^3)

	out << "Member " << m.id() << "\n"
		<< "force: " << m.force << "\n"
		<< "mass: " << mass << "\n" 
		<< "length: " << length << "\n"
		<< "width: " << width << "\n"
		<< "D: " << D << "\n"
		<< "d: " << d << "\n"
		<< "b: " << b << "\n";
}

//returns mass in grams
double massOfMember(Member m) {
	double length, width, D, d, b, areaOfTopFace;
	setMemberProperties(m, length, width, D, d, b, areaOfTopFace);
	const double density = 0.141 * 1000000;
	return (areaOfTopFace * 0.003175 * density);
}