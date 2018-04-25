#ifndef __OPENSERVOROBOT_MANIPULATOR_H
#define __OPENSERVOROBOT_MANIPULATOR_H

#include <string>
#include <vector>
#include <queue>

#include "manipulator.h"
#include "openservo_lib.h"

using namespace std;

typedef struct sv_info
{
  int servo_id;
  int AD_min;
  int AD_max;
  int AD_center;
  float faktor;
}servo_info;

JointDescription joint_description(JointType type, float dh_theta, float dh_alpha, float dh_d, float dh_a, float min, float max);

JointState joint_state(const JointDescription& joint, float position, JointStateType type = IDLE);


class OpenServoRobot 
//: public Manipulator, private OpenServo
{
public:
  //OpenServoRobot(string path_to_i2c_port, string path_to_robot_description_file);
  OpenServoRobot(string path_to_i2c_port);
  ~OpenServoRobot();

  // dodano
  int connectTo(string path_to_i2c_port);
  int loadRobotDescription(string path_to_file);
  //virtual int init(); // ?

  int lock(int joint = -1);
  int release(int joint = -1);
  int rest();

  int size();
  int move(int joint, float position, float speed);	

  ManipulatorDescription describe();
  ManipulatorState state();
  
private:
  
  enum ActionType {MOVE, UPDATE_JOINTS};
  struct buff_data
  {
    ActionType action_type;
    int joint;
    float speed;
    float position;
  };
  
  // thread 
  pthread_t thread1, thread_req;
  bool non_blocking;
  pthread_mutex_t q_mutex        = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t sleep_mutex    = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t read_servo_mutex    = PTHREAD_MUTEX_INITIALIZER;
  pthread_cond_t wake_up_condition = PTHREAD_COND_INITIALIZER;
  bool in_slipe;
  bool end_thread;
  std::queue<buff_data> q_out;
  
  ManipulatorDescription _description;
  ManipulatorState _state;
  vector<float> min_pos;
  vector<float> max_pos;
  vector<servo_info> servo;
  vector<int> joint_to_adr;
  int read_rate;
  
  OpenServo open_servo;
  
  static void* startRutine(void* arg)
  {
    OpenServoRobot* ops = reinterpret_cast<OpenServoRobot*>(arg);
    ops->threadRutine();
  }
  static void* startRutineReq(void* arg)
  {
    OpenServoRobot* ops = reinterpret_cast<OpenServoRobot*>(arg);
    ops->threadRutineReq();
  }
  void threadRutine();
  void threadRutineReq();
  
  void sendMove(int joint, float speed, float position);
  void updateJoints();
  servo_info servo_description(int id, int AD_min, int AD_max, int AD_center, float faktor);
  float scale_servo_to_joint(servo_info si, float ad_pos);
  float scale_joint_to_servo(sv tmp_sv, servo_info si, float pos);
  int joint_to_motor(int joint);
  int motor_to_joint(int m);
  
  void printServos();
  
};

#endif


/*
    Joint description:
      - min, max AD vrednosti -> dh_min in dh_max se porečunata glede na parametre
      - izhodišno verdnost za AD
      - max kot servota
      - faktor pretvorbe iz AD v kot
      - dh_theta, dh_alpha, dh_d, dh_a, dh_min(se lahko naknadno poračuna), dh_max(se lahko naknadno poračuna)

*/
