#include "Truss.h"
#include <cmath>

Joint::Joint(double x, double y):
m_x(x), 
m_y(y),
m_appliedForceX(0),
m_appliedForceY(0) {}

double Joint::getX() const{ return m_x; }
double Joint::getY() const{ return m_y; }
void Joint::setX(double x) { m_x = x; }
void Joint::setY(double y) { m_y = y; }
int Joint::numMembers() const{ return members.size(); }
void Joint::setAppliedForce(double x, double y) {
	m_appliedForceX = x;
	m_appliedForceY = y;
}
double Joint::appliedX() const{ return m_appliedForceX; }
double Joint::appliedY() const{ return m_appliedForceY; }

/* ****************
   Member Class
**************** */

int Member::m_nextId = 0;

Member::Member(Joint* left, Joint* right):
leftParent(left), 
rightParent(right),
force(0)
{
     m_id = m_nextId;
     m_nextId++;

}

double Member::length() const {
  double dy = leftParent->getY() - rightParent->getY();
  double dx = leftParent->getX() - rightParent->getX();

  return sqrt(dx*dx + dy*dy); 
}

int Member::id() const { return m_id; }
Joint* Member::leftJoint() const { return leftParent; }
Joint* Member::rightJoint() const { return rightParent; }