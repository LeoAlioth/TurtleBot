#ifndef __MANIPULATOR_MSGS_H
#define __MANIPULATOR_MSGS_H
#include <vector>
#include <string>
using namespace std;



//namespace manus {
//namespace manipulator {



enum ManipulatorStateType { PASSIVE, UNKNOWN, CONNECTED, ACTIVE, CALIBRATION };

enum JointType { ROTATION, FIXED, TRANSLATION, GRIPPER };

enum JointStateType { IDLE, MOVING, ERROR };


class JointDescription;
class JointState;
class ManipulatorDescription;
class ManipulatorState;


class JointDescription {
public:
	JointDescription() {};
	virtual ~JointDescription() {};
	JointType type;
	float dh_theta;
	float dh_alpha;
	float dh_d;
	float dh_a;
	float dh_min;
	float dh_max;
	
};

class JointState {
public:
	JointState() {};
	virtual ~JointState() {};
	JointStateType type;
	float position;
	float goal;
	float speed;
	
};


class ManipulatorDescription {
public:
	ManipulatorDescription() {};
	virtual ~ManipulatorDescription() {};
	string name;
	float version;
	std::vector<JointDescription> joints;
	
};

class ManipulatorState {
public:
	ManipulatorState() {};
	virtual ~ManipulatorState() {};
	ManipulatorStateType state;
	std::vector<JointState> joints;
	
};


//}
//}





#endif
