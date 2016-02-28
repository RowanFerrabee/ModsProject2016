

int calculateForces(vector<Members*> members, vector<Joint*> joints) {
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

int calculatePV(vector<Members*> members) {
	int sum = 0;
	for (int i=0; i<members.size(); i++)
		sum += members.at(i)->k() * members.at(i)->force() * members.at(i)->length();
	
	return sum;
}

void followGradient(vector<Members*> members, vector<Joint*> joints) {
	//increment each joint by 0.0001 in x
	//increment each joint by 0.0001 in y

}