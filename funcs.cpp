

void calculateForces(vector<Members*> members, vector<Joint*> joints) {
	Matrix forceMatrix(joints.size() * 2, members.size() + 1);

	for (int i=0; i<joints.size(); i++) {
		Joint* curJoint = joints.at(i);

		for (int j=0; j<curJoint->num_members; j++) {
			Joint* otherJoint = curJoint->members[j].leftjoint() == this ? members[j].rightjoint() : members[j].leftjoint();
			double angle = angle(curJoint, otherJoint);
			forceMatrix.setElement(2*i, curJoint->member[j].index, cos(angle));
			forceMatrix.setElement(2*i+1, curJoint->member[j].index, sin(angle));
		}
		forceMatrix.setElement(2*i, member.size(), curJoint->netForce);
		forceMatrix.setElement(2*i+1, member.size(), curJoint->netForce);
	}

	forceMatrix = forceMatrix.rref();
	for (int i=0; i<members.size(); i++) {
		members.at(i)->setForce(froceMatrix.getElement(i, members.size()));
	}
}

double calculatePV(vector<Members*> members) {
	double sum = 0;
	for (int i=0; i<members.size(); i++)
		sum += members.at(i)->k() * members.at(i)->force() * members.at(i)->length();
	
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
			joints.at(i).x += joint_increment;
			dPV_x[i] = calculatePV(members) - PV;
			joints.at(i).x -= joint_increment;

			if (i==0) max_dPV = dPV_x[i]

			joints.at(i).y += joint_increment;
			dPV_y[i] = calculatePV(members) - PV;
			joints.at(i).x -= joint_increment;

			if (dPV_x[i] > max_dPV) max_dPV = dPV_x[i];
			if (dPV_y[i] > max_dPV) max_dPV = dPV_y[i];
		}

		for (int i=0; i<joints.size(); i++) {	//Descend the gradient
			joints.at(i).x -= ascent_rate * dPV_x[i];
			joints.at(i).y -= ascent_rate * dPV_y[i];
		}
	} while(max_dPV > exit_rate);
}