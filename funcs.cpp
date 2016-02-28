/*
	When defining cantilever, define all joints, then define members between joints
	Initial test: calculate forces in members per unit P and see if correct
	Secondary test: output required widths and see if correct
	Run thru gradient descent and watch PV as is goes so we dont waste time on bad descent constants

	
*/

int main() {
	vector<Joint> joints;
	vector<Joint> movableJoints;
	joints.push(0,0.05);
	joints.push(0,0.3);
	joints.push(0,0);
}

double angle(Joint* start, Joint* end) {
	return atan2(end.getY() - start.getY(), end.getX() - start.getX());
}

void calculateForces(vector<Members*> members, vector<Joint*> joints) {
	Matrix forceMatrix(joints.size() * 2, members.size() + 1);

	for (int i=0; i<joints.size(); i++) {
		Joint* curJoint = joints.at(i);

		for (int j=0; j<curJoint->numMembers; j++) {
			Joint* otherJoint = curJoint->members[j].leftJoint() == this ? members[j].rightJoint() : members[j].leftjoint();
			double angle = angle(curJoint, otherJoint);
			forceMatrix.setElement(2*i, curJoint->member[j].id, cos(angle));
			forceMatrix.setElement(2*i+1, curJoint->member[j].id, sin(angle));
		}
		forceMatrix.setElement(2*i, member.size(), curJoint->appliedX);
		forceMatrix.setElement(2*i+1, member.size(), curJoint->appliedY);
	}

	forceMatrix = forceMatrix.rref();
	for (int i=0; i<members.size(); i++) {
		members.at(i)->force = froceMatrix.getElement(i, members.size());
	}
}

double calculatePV(vector<Members*> members) {
	double sum = 0;
	double kt = 14.4863;
	double kc = 2 * kt;

	for (int i=0; i<members.size(); i++) {
		double k = members.at(i)->force < 0 ? kc : kt;
		sum += k * members.at(i)->force * members.at(i)->length();
	}
	
	return sum;
}

void followGradient(vector<Members*> members, vector<Joint*> joints) {
	double joint_increment = 0.003;	//Make this like 3mm
	double ascent_rate = 0.1;		//THIS NEEDS SERIOUS TINKERING
	double exit_rate = 0.5;

	double PV = calculatePV(members);
	double *dPV_x = new double [joints.size()];
	double *dPV_y = new double [joints.size()];
	double max_dPV;

	do {
		for (int i=0; i<joints.size(); i++) {	//Only works if members have joint references
			joints.at(i).setX(joints.at(i).getX() + joint_increment);
			dPV_x[i] = calculatePV(members) - PV;
			joints.at(i).setX(joints.at(i).getX() - joint_increment);

			if (i==0) max_dPV = dPV_x[i]

			joints.at(i).setY(joints.at(i).getY() + joint_increment);
			dPV_y[i] = calculatePV(members) - PV;
			joints.at(i).setY(joints.at(i).getY() - joint_increment);

			if (dPV_x[i] > max_dPV) max_dPV = dPV_x[i];
			if (dPV_y[i] > max_dPV) max_dPV = dPV_y[i];
		}

		for (int i=0; i<joints.size(); i++) {	//Descend the gradient
			joints.at(i).x -= ascent_rate * dPV_x[i];
			joints.at(i).y -= ascent_rate * dPV_y[i];
		}
	} while(max_dPV > exit_rate);
}