#ifndef __MANIPULATOR_MSGS_H
#define __MANIPULATOR_MSGS_H
#include <vector>
#include <string>
using namespace std;



//namespace manus {
//namespace manipulator {



enum ManipulatorStateType { MANIPULATORSTATETYPE_UNKNOWN, MANIPULATORSTATETYPE_CONNECTED, MANIPULATORSTATETYPE_PASSIVE, MANIPULATORSTATETYPE_ACTIVE, MANIPULATORSTATETYPE_CALIBRATION };

enum JointType { JOINTTYPE_ROTATION, JOINTTYPE_TRANSLATION, JOINTTYPE_GRIPPER, JOINTTYPE_FIXED };

enum JointStateType { JOINTSTATETYPE_IDLE, JOINTSTATETYPE_MOVING, JOINTSTATETYPE_ERROR };


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
	float dh_safe;
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
