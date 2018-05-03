#include <string>
#include <cmath>

// open servo robot
#include "openservorobot.h"

// ROS
//#include <sound_play/sound_play.h>
#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Joy.h>

#include <openservorobot/JointDescriptionM.h>
#include <openservorobot/JointStateM.h>
#include <openservorobot/ManipulatorDescriptionM.h>
#include <openservorobot/ManipulatorStateM.h>
#include <openservorobot/MoveJoint.h>

using namespace std;

//--------------------------------
//        CLASS: OpenServoRobotNode
//--------------------------------

class OpenServoRobotNode
{
private:
  ros::NodeHandle node1;
  ros::Subscriber sub_joint_position_move;
  ros::Subscriber sub_get_manipulator_description;
  ros::Subscriber joy_sub_;
  ros::Publisher  pub_joint_position;
  ros::Publisher  pub_joint_description;
  tf::TransformBroadcaster br_tr;
  
  //Parameters to be read from a launch file
  string dev_i2c = "/dev/i2c-1";
  string man_yaml = "manipulator1.yaml";
  string cal_yaml = "calibration_8.yaml";
  //string prefix = "~/catkin_ws/devel/lib/openservorobot/";
  string prefix = "/home/vicos/ROS/src/openservorobot/res/";
  
  //OpenServoRobot osr("/dev/i2c-1");
  OpenServoRobot* osr;
  openservorobot::JointStateM _joint_state;
  openservorobot::ManipulatorDescriptionM _manipulator_description;
  openservorobot::ManipulatorStateM _manipulator_state;
  
  ManipulatorState _state;
  ManipulatorDescription _description;
  
  float joy_axes[6];// = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
  
  //callbacks:
  void moveJoint(const openservorobot::MoveJoint &move_joint_to);
  void getManipulatorDescription(const std_msgs::Empty &msg);
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  
public:
  OpenServoRobotNode(int joy=0)
  {
    // publish:
    pub_joint_description = node1.advertise<openservorobot::ManipulatorDescriptionM>("openservorobot/manipulator_description", 50);
    pub_joint_position    = node1.advertise<openservorobot::ManipulatorStateM>("openservorobot/manipulator_state", 100);
    
    if (joy)
    {
      cout << "** JOY is enabled." << endl;
      joy_sub_ = node1.subscribe<sensor_msgs::Joy>("joy", 10, &OpenServoRobotNode::joyCallback, this); //10
    }
    
    //cout << dev_i1c << endl;
    //cout << node1.hasParam("/openservorobot_node/dev_i1c") << endl;
    node1.getParam("/openservorobot_node/dev_i2c", dev_i2c);
    node1.getParam("/openservorobot_node/cal_yaml", cal_yaml);
    node1.getParam("/openservorobot_node/prefix", prefix);
    
    cout << dev_i2c << endl;
    cout << cal_yaml << endl;
    cout << prefix << endl;
    
    // subscrib:
    sub_joint_position_move         = node1.subscribe("openservorobot/move_joint_to", 20, &OpenServoRobotNode::moveJoint, this);
    sub_get_manipulator_description = node1.subscribe("openservorobot/get_manipulator_description", 10, &OpenServoRobotNode::getManipulatorDescription, this);
  }
  ~OpenServoRobotNode()
  {
    delete osr;
  }
  
  void initRobotArm();
  void publishJointPosition();
  void publishJointDescription();
  //void publishJointsTf();
  void publishJointsTf(float x, float y, float z, float rx, float ry, float rz, string parent, string child);
  void pubTfRobotTest();
  void joyMoveArm();
  void updateArmState();
};

//void OpenServoRobotNode::initRobotArm(string dev_i2c, string manipulator_yaml, string calibration_yaml)
void OpenServoRobotNode::initRobotArm()
{
  //osr = new OpenServoRobot("/dev/i2c-0", "manipulator1.yaml", "calibration_9.yaml");
  osr = new OpenServoRobot(dev_i2c, prefix+man_yaml, prefix+cal_yaml);
}

void OpenServoRobotNode::moveJoint(const openservorobot::MoveJoint &move_joint_to)
{
  //osr->move(move_joint_to->joint, move_joint_to->position, move_joint_to->speed);
  if (move_joint_to.joint > 3)
	osr->move(move_joint_to.joint+1, move_joint_to.position, move_joint_to.speed);
  else
    osr->move(move_joint_to.joint, move_joint_to.position, move_joint_to.speed);
}

void OpenServoRobotNode::getManipulatorDescription(const std_msgs::Empty &msg)
{
  publishJointDescription();
}

void OpenServoRobotNode::updateArmState()
{
  _state = osr->state();
}

void OpenServoRobotNode::publishJointPosition()
{
  if( osr->size()-1 != _manipulator_state.joints.size())
  {
    // prišlo je do spremembe pri številu servotov, javimo novo stanje opisa roke...
    publishJointDescription();
    _manipulator_state.joints.resize(osr->size()-1);
  }
  
  //_state = osr->state();
  _manipulator_state.state = (int)_state.state;
  //for (int q=0; q<_state.joints.size(); q++)
  //{
    //// kopiraj podatke; se da to nareditikako lepše??
    //_manipulator_state.joints[q].position = _state.joints[q].position;
    //_manipulator_state.joints[q].goal     = _state.joints[q].goal;
    //_manipulator_state.joints[q].speed    = _state.joints[q].speed;
  //}
  
  int q=0;
  int r=0;
  while(q<_manipulator_state.joints.size()){
	if (q==4) r++;
    _manipulator_state.joints[q].position = _state.joints[r].position;
    _manipulator_state.joints[q].goal     = _state.joints[r].goal;
    _manipulator_state.joints[q].speed    = _state.joints[r].speed;
    q++;
    r++;
  }
  
  pub_joint_position.publish(_manipulator_state);
}

void OpenServoRobotNode::publishJointDescription()
{
  if( osr->size()-1 != _manipulator_state.joints.size())
  {
    // prišlo je do spremembe pri številu servotov
    _manipulator_description.joints.resize(osr->size()-1);
  }
  
  _description = osr->describe();
  _manipulator_description.name   = _description.name;
  _manipulator_description.version  = _description.version;
/*  for (int q=0; q<_description.joints.size(); q++)
  {
//	if (_description.joints[q].type==JointType::JOINTTYPE_FIXED)
//		continue;
    _manipulator_description.joints[q].type     = _description.joints[q].type;
    _manipulator_description.joints[q].dh_theta = _description.joints[q].dh_theta;
    _manipulator_description.joints[q].dh_alpha = _description.joints[q].dh_alpha;
    _manipulator_description.joints[q].dh_d     = _description.joints[q].dh_d;
    _manipulator_description.joints[q].dh_a     = _description.joints[q].dh_a;
    _manipulator_description.joints[q].dh_min   = _description.joints[q].dh_min;
    _manipulator_description.joints[q].dh_max   = _description.joints[q].dh_max;
  }*/
  int q=0;
  int r=0;
  while(q<_manipulator_description.joints.size()){
	if (_description.joints[q].type==JointType::JOINTTYPE_FIXED){
		r++;
	   }
	_manipulator_description.joints[q].type     = _description.joints[r].type;
    _manipulator_description.joints[q].dh_theta = _description.joints[r].dh_theta;
    _manipulator_description.joints[q].dh_alpha = _description.joints[r].dh_alpha;
    _manipulator_description.joints[q].dh_d     = _description.joints[r].dh_d;
    _manipulator_description.joints[q].dh_a     = _description.joints[r].dh_a;
    _manipulator_description.joints[q].dh_min   = _description.joints[r].dh_min;
    _manipulator_description.joints[q].dh_max   = _description.joints[r].dh_max;
    q++;
    r++;
  }
  
  pub_joint_description.publish(_manipulator_description);
  
}

void OpenServoRobotNode::publishJointsTf(float x, float y, float z, float rx, float ry, float rz, string parent, string child)
{
  tf::Transform tr;
  tf::Quaternion q;

  tr.setOrigin( tf::Vector3(x, y, z) );
  //tr.setRotation( tf::Quaternion( 0.0, 0.0, 0.0) );
  q.setRPY( rx, ry, rz );
  tr.setRotation( q );
  //br_tr.sendTransform( tf::StampedTransform( tr, ros::Time::now(), "map", "plate_base" ) );
  br_tr.sendTransform( tf::StampedTransform( tr, ros::Time::now(), parent, child ) );
}

void OpenServoRobotNode::pubTfRobotTest()
{
  static int count = 0;
  if( osr->size() != _manipulator_state.joints.size())
  {
    // prišlo je do spremembe pri številu servotov, javimo novo stanje opisa roke...
    publishJointDescription();
    _manipulator_state.joints.resize(osr->size());
  }
  
  if( osr->size() != 7)
  {
    cout << "samo " << osr->size() << " servotov\n";
    return;
  }
  
  /*
  if( (++count % 50) == 0)
  {
    //cout << " " << count;
    cout << _state.joints[0].position << " " <<_state.joints[1].position << " " <<_state.joints[2].position << " " <<_state.joints[3].position << " " <<_state.joints[5].position << " " <<_state.joints[6].position << endl;
  }
  * */
  //_state = osr->state();
  
  publishJointsTf(1,      0,    0,    0, 0, 0, "map", "plate_base");
  publishJointsTf(0,  0.09, 0.065, 0, 0, -_state.joints[0].position+M_PI/2, "plate_base", "joint1");
  publishJointsTf(0,  0,    0.046, -M_PI/2, _state.joints[1].position+M_PI, M_PI, "joint1", "joint2");
  publishJointsTf(0.14,  0, 0,  0, 0, _state.joints[2].position, "joint2", "joint3");
  publishJointsTf(0.112, 0, 0,  0, 0, _state.joints[3].position, "joint3", "joint4");
  //publishJointsTf(0, 0, 0,  0, 0, 0, "joint4", "joint5");
  publishJointsTf(0.020,  0,  0, -_state.joints[5].position, 0, 0, "joint4", "joint5");
  //publishJointsTf(0.05,      0,    0, _state.joints[6].position, 0, 0, "joint5", "joint6");
  
  /*
  publishJointsTf(1,      0,    0,    0, 0, 0, "map", "plate_base");
  publishJointsTf(0,  0.09, 0.111, 0, 0, -_state.joints[0].position, "plate_base", "joint1");
  publishJointsTf(0,  0,    0, _state.joints[1].position-M_PI/2, 0, 0, "joint1", "joint2");
  publishJointsTf(0,  0,   0.140,  _state.joints[2].position, 0, 0, "joint2", "joint3");
  publishJointsTf(0,  0,   0.112,  _state.joints[3].position, 0, 0, "joint3", "joint4");
  publishJointsTf(0,  0,   0.020,  0, 0, -_state.joints[5].position, "joint4", "joint5");
  //publishJointsTf(0.05,      0,    0, _state.joints[6].position, 0, 0, "joint5", "joint6");
  */
}

void pubTfTest(OpenServoRobotNode* osrn)
{
  osrn->publishJointsTf(1,      0,    0,    0, 0, 0, "map", "plate_base");
  //osrn->publishJointsTf(0,      0.09, 0.111, M_PI/2, 0, 0, "plate_base", "joint1");
  osrn->publishJointsTf(0,      0.09, 0.111, 0, 0, 0, "plate_base", "joint1");
  osrn->publishJointsTf(0.140,  0,    0,    0, 0, 0, "joint1", "joint2");
  osrn->publishJointsTf(0.112,   0,   0,    0, 0, 0, "joint2", "joint3");
  osrn->publishJointsTf(0.020,   0,   0,    0, 0, 0, "joint3", "joint4");
  osrn->publishJointsTf(0,      0,    0,    0, 0, 0, "joint4", "joint5");
  osrn->publishJointsTf(0.05,      0,    0, 0, 0, 0, "joint5", "joint6");
}

// namenjeno le za hitro in enostavno tesiranje manipulatorja
#define SMOOTH 0.1f
void OpenServoRobotNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{ 
  //joy->axes[];
  //joy->buttons[];
  joy_axes[0] = joy->axes[0];
  joy_axes[1] = joy->axes[1];
  joy_axes[2] = joy->axes[4];
  joy_axes[3] = joy->axes[3];
  joy_axes[4] = joy->axes[6];
  joy_axes[5] = joy->axes[7];
}

void OpenServoRobotNode::joyMoveArm()
{ 

  //_state = osr->state();
  
  osr->move(0, _state.joints[0].goal - joy_axes[0]*SMOOTH, 1);   // J1
  osr->move(1, _state.joints[1].goal - joy_axes[1]*SMOOTH, 1);   // J2
  osr->move(2, _state.joints[2].goal - joy_axes[2]*SMOOTH, 1);   // J3
  osr->move(3, _state.joints[3].goal + joy_axes[3]*SMOOTH, 1);   // J4
  osr->move(5, _state.joints[5].goal + joy_axes[4]*SMOOTH, 1);   // J5
  osr->move(6, _state.joints[6].goal + joy_axes[5]*SMOOTH, 1);   // J6
  
}


//----------------------------------------------------------------------
//    MAIN
//----------------------------------------------------------------------

string GetCurrentWorkingDir( void ) {
  char buff[64];
  getcwd( buff, 64 );
  std::string current_working_dir(buff);
  return current_working_dir;
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "openservorobot_node");
  
  int joy = 0;
/*  
  cout << "** current working dir: " << GetCurrentWorkingDir() << endl;
  
  for (int q=1; q<argc; q++)
  {
    if (!string(argv[q]).compare("-d"))
    {
      if (argc < q+1)
        break;
      dev_i2c = string(argv[q+1]);
      cout << "new i2c device = " << dev_i2c << endl;
      continue;
    }
    if (!string(argv[q]).compare("-m"))
    {
      if (argc < q+1)
        break;
      man_yaml = string(argv[q+1]);
      cout << "new manipulator yaml = " << man_yaml << endl;
      continue;
    }
    if (!string(argv[q]).compare("-c"))
    {
      if (argc < q+1)
        break;
      cal_yaml = string(argv[q+1]);
      cout << "new calibration yaml = " << cal_yaml << endl;
    }
    if (!string(argv[q]).compare("-j"))
      joy = 1;
      
  }
  
  cout << "** USING:" << endl;
  cout << " - i2c device = " << dev_i2c << endl;
  cout << " - manipulator yaml = " << man_yaml << endl;
  cout << " - calibration yaml = " << cal_yaml << endl;
  */
  //return 0;
  
  OpenServoRobotNode osrn(joy);
  osrn.initRobotArm();
  
  osrn.publishJointDescription();
  
  ros::Rate rate(10);
  //int n = 1;
  while(ros::ok())
  {
	  
	ros::spinOnce();
	//cout << "spinOnce, " << flush;
    //cout << "updateArm, " << flush;

    osrn.updateArmState();
    
    osrn.publishJointPosition();
    osrn.publishJointDescription();
    ///osrn.pubTfRobotTest();
    
    if (joy)
      osrn.joyMoveArm();

    //cout << n <<" ros loop" << endl;
    //n++;
    rate.sleep();
  }
  return 0;
	
}
