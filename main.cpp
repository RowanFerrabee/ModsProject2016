/*
	When defining cantilever, define all joints, then define members between joints
	Initial test: calculate forces in members per unit P and see if correct
	Secondary test: output required widths and see if correct
	Run thru gradient descent and watch PV as is goes so we dont waste time on bad descent constants
*/

#include "Truss.h"
#include "matrix.h"
#include <cmath>

using namespace std;

void calculateForces(vector<Member*> members, vector<Joint*> joints);
double calculatePV(vector<Member*> members);
void followGradient(vector<Member*> members, vector<Joint*> joints);

int main() {
	vector<Joint*> joints;
	vector<Joint*> movableJoints;
	vector<Member*> members;

	cout << "Point 0: (0,0.05)" << endl;
	cout << "Point 1: (0,0.0)" << endl;
	cout << "Point 2: (0.3,0)" << endl;
	joints.push_back(new Joint(0,0.05));
	joints.push_back(new Joint(0,0));
	joints.push_back(new Joint(0.3,0));


	joints[0]->setAppliedForce(-6, 1);
	joints[1]->setAppliedForce(6, 0);
	joints[2]->setAppliedForce(0, -1);

	double x, y;
	int counter = 3;
	cout << "Insert Points Now" << endl;
	do {
		cout << "Point " << counter << ": ";
		cin >> x >> y;
		if (x > 0) {
			joints.push_back(new Joint(x,y));
			movableJoints.push_back(new Joint(x,y));
		}
		counter++;
	} while (x > 0);

	int joint1, joint2;
	cout << "Insert Pair of joints to draw members" << endl;
	do {
		cin >> joint1 >> joint2;
		if (joint1 >= 0 && joint2 >= 0 && joint1 < joints.size() && joint2 < joints.size()) {
			Member* newMember = new Member(joints[joint1], joints[joint2]);
			members.push_back(newMember);
			joints[joint1]->members.push_back(newMember);
			joints[joint2]->members.push_back(newMember);
		}
	} while (joint1 >= 0 && joint2 >= 0);

	calculateForces(members, joints);

	for (int i=0; i<members.size(); i++) {
		cout << "Member " << members[i]->id() << " has force: " << members[i]->force << endl;
	}

	cout << "The PV of this bridge is: " << calculatePV(members) << endl;

}

void calculateForces(vector<Member*> memberList, vector<Joint*> joints) {
	Matrix forceMatrix(joints.size() * 2, memberList.size() + 1);
	//Be sure that the members vector is sorted by id

	if(joints.size()*2 < memberList.size() + 1) {
		cout << "Bridge makes no sense, force calculations make no sense." << endl;
		return;
	}

	for (int i=0; i<joints.size(); i++) {
		Joint* curJoint = joints.at(i);
		//cout << "Joint " << i << ": (" << curJoint->getX() << "," << curJoint->getY() << ")" << endl;
		//cout << "Has " << curJoint->numMembers() << " members" << endl;
		//cout << "Those members are: ";
		for (int i=0; i<curJoint->numMembers(); i++) {
			cout << curJoint->members[i]->id() << " ";
		} cout << endl;

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

double calculatePV(vector<Member*> members) {
	double sum = 0;
	double kt = 14.4863;
	double kc = -1 * 2 * kt;

	for (int i=0; i<members.size(); i++) {
		double k = members.at(i)->force < 0 ? kc : kt;
		sum += k * members.at(i)->force * members.at(i)->length();
	}
	
	return sum;
}

void followGradient(vector<Member*> members, vector<Joint*> joints) {
	double joint_increment = 0.003;	//Make this like 3mm
	double ascent_rate = 0.1;		//THIS NEEDS SERIOUS TINKERING
	double exit_rate = 0.5;

	double PV = calculatePV(members);
	double *dPV_x = new double [joints.size()];
	double *dPV_y = new double [joints.size()];
	double max_dPV;

	do {
		for (int i=0; i<joints.size(); i++) {	//Only works if members have joint references
			joints.at(i)->setX(joints.at(i)->getX() + joint_increment);
			dPV_x[i] = calculatePV(members) - PV;
			joints.at(i)->setX(joints.at(i)->getX() - joint_increment);

			if (i==0) max_dPV = dPV_x[i];

			joints.at(i)->setY(joints.at(i)->getY() + joint_increment);
			dPV_y[i] = calculatePV(members) - PV;
			joints.at(i)->setY(joints.at(i)->getY() - joint_increment);

			if (dPV_x[i] > max_dPV) max_dPV = dPV_x[i];
			if (dPV_y[i] > max_dPV) max_dPV = dPV_y[i];
		}

		for (int i=0; i<joints.size(); i++) {	//Descend the gradient
			joints.at(i)->setX(joints.at(i)->getX() - ascent_rate * dPV_x[i]);
			joints.at(i)->setY(joints.at(i)->getY() - ascent_rate * dPV_y[i]);
		}
	} while(max_dPV > exit_rate);
}