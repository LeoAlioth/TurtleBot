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
  ros::Publisher  pub_joint_position;
  ros::Publisher  pub_joint_description;
  tf::TransformBroadcaster br_tr;
  
  //OpenServoRobot osr("/dev/i2c-1");
  OpenServoRobot* osr;
  openservorobot::JointStateM _joint_state;
  openservorobot::ManipulatorDescriptionM _manipulator_description;
  openservorobot::ManipulatorStateM _manipulator_state;
  
  ManipulatorState _state;
  ManipulatorDescription _description;
  
  //callbacks:
  void moveJoint(const openservorobot::MoveJoint &move_joint_to);
  void getManipulatorDescription(const std_msgs::Empty &msg);
  
public:
  OpenServoRobotNode()
  {
    osr = new OpenServoRobot("/dev/i2c-0");
    
    // publish:
    pub_joint_description = node1.advertise<openservorobot::ManipulatorDescriptionM>("openservorobot/manipulator_description", 50);
    pub_joint_position    = node1.advertise<openservorobot::ManipulatorStateM>("openservorobot/manipulator_state", 100);
    
    // subscrib:
    sub_joint_position_move         = node1.subscribe("openservorobot/move_joint_to", 100, &OpenServoRobotNode::moveJoint, this);
    sub_get_manipulator_description = node1.subscribe("openservorobot/get_manipulator_description", 10, &OpenServoRobotNode::getManipulatorDescription, this);
  }
  ~OpenServoRobotNode()
  {
    delete osr;
  }
  
  void publishJointPosition();
  void publishJointDescription();
  //void publishJointsTf();
  void publishJointsTf(float x, float y, float z, float rx, float ry, float rz, string parent, string child);
  void pubTfRobotTest();
};

void OpenServoRobotNode::moveJoint(const openservorobot::MoveJoint &move_joint_to)
{
  //osr->move(move_joint_to->joint, move_joint_to->position, move_joint_to->speed);
  osr->move(move_joint_to.joint, move_joint_to.position, move_joint_to.speed);
}

void OpenServoRobotNode::getManipulatorDescription(const std_msgs::Empty &msg)
{
  publishJointDescription();
}

void OpenServoRobotNode::publishJointPosition()
{
  if( osr->size() != _manipulator_state.joints.size())
  {
    // prišlo je do spremembe pri številu servotov, javimo novo stanje opisa roke...
    publishJointDescription();
    _manipulator_state.joints.resize(osr->size());
  }
  
  _state = osr->state();
  _manipulator_state.state = (int)_state.state;
  for (int q=0; q<_state.joints.size(); q++)
  {
    // kopiraj podatke; se da to nareditikako lepše??
    _manipulator_state.joints[q].position = _state.joints[q].position;
    _manipulator_state.joints[q].goal     = _state.joints[q].goal;
    _manipulator_state.joints[q].speed    = _state.joints[q].speed;
  }
  pub_joint_position.publish(_manipulator_state);
}

void OpenServoRobotNode::publishJointDescription()
{
  if( osr->size() != _manipulator_state.joints.size())
  {
    // prišlo je do spremembe pri številu servotov
    _manipulator_description.joints.resize(osr->size());
  }
  
  _description = osr->describe();
  _manipulator_description.name   = _description.name;
  _manipulator_description.version  = _description.version;
  for (int q=0; q<_description.joints.size(); q++)
  {
    _manipulator_description.joints[q].type     = _description.joints[q].type;
    _manipulator_description.joints[q].dh_theta = _description.joints[q].dh_theta;
    _manipulator_description.joints[q].dh_alpha = _description.joints[q].dh_alpha;
    _manipulator_description.joints[q].dh_d     = _description.joints[q].dh_d;
    _manipulator_description.joints[q].dh_a     = _description.joints[q].dh_a;
    _manipulator_description.joints[q].dh_min   = _description.joints[q].dh_min;
    _manipulator_description.joints[q].dh_max   = _description.joints[q].dh_max;
  }
  pub_joint_description.publish(_manipulator_description);
  
}

void OpenServoRobotNode::publishJointsTf(float x, float y, float z, float rx, float ry, float rz, string parent, string child)
{
  tf::Transform tr;
  tf::Quaternion q;
  /*
  _description = osr->describe();
  for (int q=0; q<_description.joints.size(); q++)
  {
    
  }
  */
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
  _state = osr->state();
  
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


//----------------------------------------------------------------------
//    MAIN
//----------------------------------------------------------------------
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "openservorobot_node");
  OpenServoRobotNode osrn;
  
  ros::Rate rate(10);
  while(ros::ok())
  {
    ros::spinOnce();
    // kliči še update funkcijo oz. publish...
    //osrn.publishJointsTf();
    //pubTfTest(&osrn);
    osrn.pubTfRobotTest();
    rate.sleep();
  }
  return 0;
	
}
