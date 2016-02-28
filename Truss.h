#include <vector>

class Member;

class Joint {
public:
  Joint(double x, double y);
  double getX()const;
  double getY()const;
  void setX(double x);
  void setY(double y);
  int numMembers()const;
  void setAppliedForce(double x, double y);
  double appliedX()const;
  double appliedY()const;

  std::vector<Member*> members;

private:
  double m_x;
  double m_y;
  double m_appliedForceX;  
  double m_appliedForceY;

};

class Member {
private:
  Joint* leftParent;
  Joint* rightParent;
  static int m_nextId;
  int m_id;
public:
  double force;
  Member(Joint* left, Joint* right);
  double length()const;
  int id()const;
  Joint* leftJoint()const;
  Joint* rightJoint()const;
};