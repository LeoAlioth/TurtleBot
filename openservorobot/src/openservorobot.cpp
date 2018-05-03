#include <cmath>
#include <chrono>
#include <numeric>
#include <unistd.h>

#include "openservo_lib.h"
#include "openservorobot.h"

#include <yaml-cpp/yaml.h>

using namespace std;

#define RADIAN_TO_DEGREE(X) ((X * 180) / M_PI )
#define DEGREE_TO_RADIAN(X) ((X / 180) * M_PI )

JointDescription joint_description(JointType type, float dh_theta, float dh_alpha, float dh_d, float dh_a, float min, float max) {
  JointDescription joint;
  joint.type = type;
  joint.dh_theta = DEGREE_TO_RADIAN(dh_theta);
  joint.dh_alpha = DEGREE_TO_RADIAN(dh_alpha);
  //joint.dh_theta = dh_theta;
  //joint.dh_alpha = dh_alpha;
  joint.dh_d = dh_d;
  joint.dh_a = dh_a;
  //joint.dh_min = type == JointType::ROTATION ? DEGREE_TO_RADIAN(min) : min;
  //joint.dh_max = type == JointType::ROTATION ? DEGREE_TO_RADIAN(max) : max;
  joint.dh_min = min;
  joint.dh_max = max;
  return joint;
}

JointState joint_state(const JointDescription& joint, float position, JointStateType type) {
  JointState state;
  state.type = type;
  state.position = joint.type == JointType::JOINTTYPE_ROTATION ? DEGREE_TO_RADIAN(position) : position;
  state.goal = joint.type == JointType::JOINTTYPE_ROTATION ? DEGREE_TO_RADIAN(position) : position;
  return state;
}

bool parse_joint (const YAML::Node& node, JointDescription& joint) {
    string type = node["type"].as<string>();
    if (type == "rotation") {
        joint.type = JOINTTYPE_ROTATION;
        joint.dh_alpha = DEGREE_TO_RADIAN(node["dh"]["alpha"].as<float>());
        joint.dh_d = node["dh"]["d"].as<float>();
        joint.dh_a = node["dh"]["a"].as<float>();
        joint.dh_min = DEGREE_TO_RADIAN(node["min"].as<float>());
        joint.dh_max = DEGREE_TO_RADIAN(node["max"].as<float>());
        joint.dh_safe = DEGREE_TO_RADIAN(node["safe"].as<float>());

        joint.dh_theta = joint.dh_safe;
        return true;
    }
    if (type == "translation") {
        joint.type = JOINTTYPE_TRANSLATION;
        joint.dh_theta = DEGREE_TO_RADIAN(node["dh"]["theta"].as<float>());
        joint.dh_alpha = DEGREE_TO_RADIAN(node["dh"]["alpha"].as<float>());
        joint.dh_a = node["dh"]["a"].as<float>();
        joint.dh_min = node["min"].as<float>();
        joint.dh_max = node["max"].as<float>();
        joint.dh_safe = node["safe"].as<float>();

        joint.dh_d = joint.dh_safe;
        return true;
    }
    if (type == "fixed") {
        joint.type = JOINTTYPE_FIXED;
        joint.dh_theta = DEGREE_TO_RADIAN(node["dh"]["theta"].as<float>());
        joint.dh_alpha = DEGREE_TO_RADIAN(node["dh"]["alpha"].as<float>());
        joint.dh_d = node["dh"]["d"].as<float>();
        joint.dh_a = node["dh"]["a"].as<float>();
        joint.dh_min = 0;
        joint.dh_max = 0;
        joint.dh_safe = 0;
        return true;
    }
    if (type == "gripper") {
        joint.type = JOINTTYPE_GRIPPER;
        joint.dh_theta = 0;
        joint.dh_alpha = 0;
        joint.dh_d = node["grip"].as<float>();
        joint.dh_a = 0;
        joint.dh_min = node["min"].as<float>();
        joint.dh_max = node["max"].as<float>();
        joint.dh_safe = 0;
        return true;
    }
    return false;
}

bool parse_description(const string& filename, ManipulatorDescription& manipulator) {

    YAML::Node doc = YAML::LoadFile(filename);

    manipulator.name = doc["name"].as<string>();
    manipulator.version = doc["version"].as<float>();

    const YAML::Node& joints = doc["joints"];
    manipulator.joints.clear();

    for (int i = 0; i < joints.size(); i++) {
        JointDescription d;
        parse_joint(joints[i], d);
        manipulator.joints.push_back(d);
    }

    return true;
}

//**********************************************************************

// le začasno, za debagiranje!
void printServoRegisters(sv* servo)
{
  cout << "\t- type: " << servo->type << endl;          // OpenServo device type
  cout << "\t- subtype: " << servo->subtype << endl;        // OpenServo device subtype
  cout << "\t- version: " << servo->version << endl;        // version number of OpenServo software
  cout << "\t- flags: " << servo->flags << endl;
  cout << "\t- timer: " << servo->timer << endl;          // Timer -­ incremented each ADC sample
  cout << "\t- position: " << servo->position << endl;        // Servo position
  cout << "\t- velocity: " << servo->velocity << endl;        // Servo velocity
  cout << "\t- current: " << servo->current << endl;        // Servo current/power
  cout << "\t- pwm_cw: " << servo->pwm_cw << endl;          // PWM clockwise value
  cout << "\t- pwm_ccw: " << servo->pwm_ccw << endl;        // PWM counter­clockwise value
  //cout << "\t- speed: " << servo->speed << endl;
  
  // read/write (7 values)
  cout << "\t- seek_position: " << servo->seek_position << endl;      // Seek position
  cout << "\t- seek_velocity: " << servo->seek_velocity << endl;      // Speed seek position
  cout << "\t- voltage: " << servo->voltage << endl;        // Battery/adapter Voltage value
  cout << "\t- curve_delta: " << servo->curve_delta << endl;      // Curve Time delta
  cout << "\t- curve_position: " << servo->curve_position << endl;      // Curve position
  cout << "\t- curve_in_velocity: " << servo->curve_in_velocity << endl;    // Curve in velocity
  cout << "\t- curve_out_velocity: " << servo->curve_out_velocity << endl;    // Curve out velocity
  
  // read/write protected (9 values)
  cout << "\t- address: " << servo->address << endl;        // TWI address of servo
  cout << "\t- deadband: " << servo->deadband << endl;        // Programmable PID deadband value
  cout << "\t- pgain: " << servo->pgain << endl;          // PID proportional gain
  cout << "\t- dgain: " << servo->dgain << endl;          // PID derivative gain
  cout << "\t- igain: " << servo->igain << endl;          // PID integral gain
  cout << "\t- pwm_freq_divider: " << servo->pwm_freq_divider << endl;    // PWM frequency divider
  cout << "\t- minseek: " << servo->minseek << endl;        // Minimum seek position
  cout << "\t- maxseek: " << servo->maxseek << endl;        // Maximum seek position
  cout << "\t- reverse_seek: " << servo->reverse_seek << endl;      // reverse seek mode
}
// le začasno, za debagiranje!
void OpenServoRobot::printServos()
{
  cout << "scanning port:\n";
  unsigned char adrs[128];
  int num = open_servo.scanPort(adrs);
  cout << "\t- found " << num << " devicess\n";
  cout << "\t- adress:";
  for (int q=0; q<num; q++)
  {
    cout << " " << (unsigned int)adrs[q];
  }
  cout << endl;
  
  cout << "auto adding servos:\n";
  num = open_servo.scanPortAutoAddServo();
  cout << "\t- added " << num << " servos\n";
  cout << "\t- vector size: " << open_servo.getNumOfServos() << endl;
  for (int q=0, w = open_servo.getNumOfServos(); q<w; q++)
  {
    cout << "* SERVO: " << q << ", adr " << (unsigned int)adrs[q] << endl;
    sv tmp_sv = open_servo.getServo((unsigned int)adrs[q]);
    if (&tmp_sv != NULL)
      printServoRegisters(&tmp_sv);
    //printServoRegisters(&my_servo.getServo((unsigned int)adrs[q]));
  }
  
}

float scale_servo_to_joint(servo_info si, float ad_pos)
{
  float pos_deg = (ad_pos - si.AD_center) * si.factor;
  return round(pos_deg * 100.0) / 100.0;
}

float scale_joint_to_servo(sv tmp_sv, servo_info si, float pos)
{
  float pos_ad = (pos/si.factor) + si.AD_center;
  return pos_ad;
}


//OpenServoRobot::OpenServoRobot(string path_to_i2c_port)
OpenServoRobot::OpenServoRobot(string path_to_i2c_port, const string& modelfile, const string& calibfile)
{
  cout << "** Open servo robot: constructor\n";
  read_rate = 20;
  _state.state = MANIPULATORSTATETYPE_UNKNOWN;
  _description.name = "i2c Robot Manipulator";
  _description.version = 0.2f;
  
  if (connectTo(path_to_i2c_port))
    _state.state = MANIPULATORSTATETYPE_ACTIVE;
  
  int num = open_servo.scanPortAutoAddServo();
  cout << "   num. of servos is " << num << endl;
  
  //printServos();
  
  //loadRobotDescription("lalalend");
  loadDescription(modelfile, calibfile);
  
  if (num < servos.size())
    cout << "** Not enough motors detected!" << endl;
  //else
  end_thread = false;
  
  pthread_create(&thread1, 0, startRutine, this);
  pthread_create(&thread_req, 0, startRutineReq, this);
}

OpenServoRobot::~OpenServoRobot()
{
  bool tmp_in_slip;
  pthread_mutex_lock(&sleep_mutex); // potrebno?
  end_thread = true;
  tmp_in_slip = in_slipe;
  pthread_mutex_unlock(&sleep_mutex);
  if (tmp_in_slip)
    pthread_cond_signal(&wake_up_condition);
  
  pthread_join(thread_req, 0);
  pthread_join(thread1, 0);
}

int OpenServoRobot::connectTo(string path_to_i2c_port)
{
  return open_servo.openPort(path_to_i2c_port) == 1;
}

/*
servo_info OpenServoRobot::servo_description(int id, int AD_min, int AD_max, int AD_center, float factor)
{
  servo_info tmp;
  tmp.servo_id = id;
  tmp.AD_min = AD_min;
  tmp.AD_max = AD_max;
  tmp.AD_center = AD_center;
  tmp.factor = factor;
  return tmp;
}

int OpenServoRobot::loadRobotDescription(string path_to_file)
{
  int fixed_joint_id = 4;
  // basic joint data and information ... HARDCODED!
  _state.joints.resize(7);
  for (int i = 0; i < 7; i++) {
		_state.joints[i].type = IDLE;
	}
  
  joint_to_adr.push_back(17);
  joint_to_adr.push_back(18);
  joint_to_adr.push_back(19);
  joint_to_adr.push_back(20);
  //joint_to_adr.push_back(0);
  joint_to_adr.push_back(21);
  joint_to_adr.push_back(22);
  
  // ti podatki se morajo prebrati iz datoteke in preveriti če se min in max ujemajo s servotom
  //servo_description(int id, int AD_min, int AD_max, int AD_center, float factor) (factor in rad.)
  servo.push_back(servo_description(0, 208, 3938, 1968, 0.0009380807));
  servo.push_back(servo_description(1, 344, 4024,  632, 0.0009349769));
  servo.push_back(servo_description(2, 226, 3922, 2880, 0.0009304994));
  servo.push_back(servo_description(3, 422, 4076, 2400, 0.0008597145));
  //servo.push_back(servo_description(4, 0, 0, 0, 0));
  servo.push_back(servo_description(5,  16, 4076, 2196, 0.0008597145));
  servo.push_back(servo_description(6,  94, 1640,  560, 0.0008597145));

	//_state.state = UNKNOWN;

	min_pos.resize(7);
	max_pos.resize(7);
  _description.joints.resize(7);
  
  //JointDescription joint_description(JointType type, float dh_theta, float dh_alpha, float dh_d, float dh_a, float min, float max)
  // min in max se (lahko) pri rokah razlikujeta
  // Joint 0, min: -1.69, max: 1.73
  //cout << "** Joint 0, min: " << scale_servo_to_joint(servo[0], servo[0].AD_min) << ", max: " << scale_servo_to_joint(servo[0], servo[0].AD_max) << endl;
	_description.joints[0] = joint_description(ROTATION,    0, 90, 111,   0, scale_servo_to_joint(servo[0], servo[0].AD_min), scale_servo_to_joint(servo[0], servo[0].AD_max));
	_description.joints[1] = joint_description(ROTATION,   140,  0,  0, 108, scale_servo_to_joint(servo[1], servo[1].AD_min), scale_servo_to_joint(servo[1], servo[1].AD_max));
	_description.joints[2] = joint_description(ROTATION,   -80, 	0,  0, 112,scale_servo_to_joint(servo[2], servo[2].AD_min), scale_servo_to_joint(servo[2], servo[2].AD_max));
	_description.joints[3] = joint_description(ROTATION,    -50, 	0,  0,  20,scale_servo_to_joint(servo[3], servo[3].AD_min), scale_servo_to_joint(servo[3], servo[3].AD_max));
	_description.joints[4] = joint_description(FIXED,       90, 90,  0,   0, 0, 0);
	_description.joints[5] = joint_description(ROTATION,    0,  0,  0,  0,   scale_servo_to_joint(servo[4], servo[4].AD_min), scale_servo_to_joint(servo[4], servo[4].AD_max));
	_description.joints[6] = joint_description(GRIPPER,     0,  0,  50,  0,  scale_servo_to_joint(servo[5], servo[5].AD_min), scale_servo_to_joint(servo[5], servo[5].AD_max));
  
  return 1;
}
*/

bool parse_calibration(const string& filename, vector<servo_info>& servos) {

  YAML::Node doc = YAML::LoadFile(filename);
  servos.clear();

  for (int i = 0; i < doc.size(); i++) {
    servo_info d;

    d.servo_id = doc[i]["id"].as<int>();
    d.joint_id = -1;
    d.AD_min = doc[i]["min"].as<int>();
    d.AD_max = doc[i]["max"].as<int>();
    d.AD_center = doc[i]["center"].as<int>();
    d.factor = doc[i]["factor"].as<float>();

    servos.push_back(d);
  }

  return true;
}

int OpenServoRobot::loadDescription(const string& modelfile, const string& calibfile) {

  cout << "-- model file: " << modelfile << endl;
  
  if (!parse_description(modelfile, _description)) {
    //throw ManipulatorException("Unable to parse manipulator model description");
    cout << "Unable to parse manipulator model description" << endl;
  }

  cout << "-- calib file: " << calibfile << endl;
  
  if (!parse_calibration(calibfile, servos)) {
    //throw ManipulatorException("Unable to parse manipulator calibration description");
    cout << "Unable to parse manipulator calibration description" << endl;
  }
  
  cout << "-- file: end" << endl;
  
  _state.joints.resize(_description.joints.size());
  int j = 0;
  for (int i = 0; i < _description.joints.size(); i++) {
    _state.joints[i].type = JOINTSTATETYPE_IDLE;

    if (_description.joints[i].type == JOINTTYPE_FIXED)
      continue;

    if (j >= servos.size())
      //throw ManipulatorException("Not enough motors in calibration data.");
      cout << "Not enough motors in calibration data." << endl;

    runtime_data.resize(runtime_data.size()+1);
    runtime_data[runtime_data.size()-1].address = servos[j].servo_id;
    servos[j].joint_id = i;
    _description.joints[i].dh_min = scale_servo_to_joint(servos[j], servos[j].AD_min);
    _description.joints[i].dh_max = scale_servo_to_joint(servos[j], servos[j].AD_max);

    cout <<  "Verifying min-max data." << endl;
    sv s = open_servo.getServo(servos[j].servo_id);
    cout <<  "Got servo id" << endl;

    if (s.minseek != servos[j].AD_min || s.maxseek != servos[j].AD_max) {

      cout << "Detected incorrect parameters, writing min-max data to motor " << i << endl;
      open_servo.writeEnable(servos[j].servo_id);
      open_servo.setMaxSeek(servos[j].servo_id, servos[j].AD_max);
      open_servo.setMinSeek(servos[j].servo_id, servos[j].AD_min);
      open_servo.writeDisable(servos[j].servo_id);
      open_servo.registerSave(servos[j].servo_id);
    }

    j++;
  }

  cout << "Joints: " << _description.joints.size() << " Motors: " << runtime_data.size() << endl;

  if (j != servos.size())
    //throw ManipulatorException("Unassigned motors remaining.");
    cout << "Unassigned motors remaining." << endl;

  //JointDescription joint_description(JointType type, float dh_theta, float dh_alpha, float dh_d, float dh_a, float min, float max)
  // min in max se (lahko) pri rokah razlikujeta
  // Joint 0, min: -1.69, max: 1.73
  //cout << "** Joint 0, min: " << scale_servo_to_joint(servos[0], servos[0].AD_min) << ", max: " << scale_servo_to_joint(servos[0], servos[0].AD_max) << endl;
  return 1;
}

int OpenServoRobot::lock(int joint)
{
	//return enableMotors(joint);
  return 1;
}

int OpenServoRobot::release(int joint)
{
	//return disableMotors(joint);
  return 1;
}

int OpenServoRobot::rest() {
	//if ( _internal.connection.connected == 0 ) return 0;
	return false;
}

int OpenServoRobot::get_file_descriptor()
{
  return -1;
}

bool OpenServoRobot::handle_input()
{
  return true;
}

bool OpenServoRobot::handle_output()
{
  return true;
}

void OpenServoRobot::disconnect()
{

}

int OpenServoRobot::size() {

	return _description.joints.size();

}


int OpenServoRobot::move(int joint, float position, float speed)
{
  //cout << " move joint " << joint << " pos: " << position << endl;
  // dodamo ukaz v vrsto za pošiljanje...
  bool tmp_in_slip;
  buff_data tmp;
  tmp.action_type = MOVE;
  tmp.joint = joint;
  tmp.speed = speed;
  tmp.position = position;
  
  pthread_mutex_lock (&q_mutex);
  q_out.push(tmp);
  pthread_mutex_unlock (&q_mutex);
  
  pthread_mutex_lock(&sleep_mutex);
  tmp_in_slip = in_slipe;
  pthread_mutex_unlock(&sleep_mutex);
  if (tmp_in_slip)
    pthread_cond_signal(&wake_up_condition);
    //cout << "cond signal wake" << endl;
    //pthread_cond_broadcast(&wake_up_condition);

  return 1;
}

#define MEDIAN_WINDOW 20

void OpenServoRobot::sendMove(int joint, float speed, float position)
{
  //
  int tmp_mot = joint_to_motor(joint);
  if (tmp_mot < 0)
    return;
  //sv tmp_sv = open_servo.getServo(joint_to_adr[tmp_mot]);
  sv tmp_sv = open_servo.getServo(runtime_data[tmp_mot].address);
  if (&tmp_sv == NULL)
    return;
  int pos = (int)::round(scale_joint_to_servo(tmp_sv, servos[tmp_mot], position));
  /// preveri, če je slučajno pos prevelik!!
  if (pos < tmp_sv.minseek)
    pos = tmp_sv.minseek;
  else if (pos > tmp_sv.maxseek)
    pos = tmp_sv.maxseek;
   
  //cout << " send move joint " << joint << " motor: " << tmp_mot << " pos: " << (int)pos << endl;
  //open_servo.setSeekPossition(joint_to_adr[tmp_mot], (int)pos);
  open_servo.setSeekPossition(runtime_data[tmp_mot].address, pos);
  
  runtime_data[tmp_mot].goal_median.clear();
  runtime_data[tmp_mot].goal_median.assign(MEDIAN_WINDOW, pos);
}	

ManipulatorDescription OpenServoRobot::describe()
{
	return _description;
}


ManipulatorState OpenServoRobot::state() 
{
  //refresh state data
  int tmp_mot = -1;
  pthread_mutex_lock(&read_servo_mutex);
  for(int q=0; q<_description.joints.size(); q++)
  {
    tmp_mot = joint_to_motor(q);
    if( tmp_mot < 0)
      continue;
    //cout << " state: q "<< q << flush;
    //pthread_mutex_lock(&read_servo_mutex);
    //sv tmp_sv = open_servo.getServo(joint_to_adr[tmp_mot]); // return null if servo not present
    sv tmp_sv = open_servo.getServo(runtime_data[tmp_mot].address); // return null if servo not present
    //pthread_mutex_unlock(&read_servo_mutex);
    if (&tmp_sv != NULL)
    {
      _state.joints[q].position = scale_servo_to_joint(servos[tmp_mot], (float)tmp_sv.position);
      _state.joints[q].goal = scale_servo_to_joint(servos[tmp_mot], (float)tmp_sv.seek_position);
      _state.joints[q].speed = 1;
      /*
      if(izpis){
        cout << "** joint: " << q << ", motor: " << tmp_mot << endl;
        cout << "    position = " << _state.joints[q].position << ", ro: " << tmp_sv.position << endl;
        cout << "    goal = " << _state.joints[q].goal << ", ro: " << tmp_sv.seek_position << endl;
        cout << "    goal = " << scale_servo_to_joint(servo[tmp_mot], (float)tmp_sv.seek_position) << endl;
        cout << "    speed = " << _state.joints[q].speed << ", ro: " << tmp_sv.velocity << endl;
      }
      */
    }
    //else
		//cout << " NULL" << flush;
    
  }
  pthread_mutex_unlock(&read_servo_mutex);
  //cout << endl;
  return _state;
}

int OpenServoRobot::joint_to_motor(int j)
{
	if (j < 0 || j >= _description.joints.size()) return -1;

	if (_description.joints[j].type == JOINTTYPE_FIXED) return -1;

	int m = 0;
	for (int i = 0; i < j; i++) {
		if (_description.joints[i].type != JOINTTYPE_FIXED)
			m++;
	}

	return m;

}

int OpenServoRobot::motor_to_joint(int m)
{
	int mt = m;
	int j = 0;
	while (m > 0) {
		j++; m--;
		if (_description.joints[j].type == JOINTTYPE_FIXED)
			j++;
	}

	return j;

}

/*
void OpenServoRobot::push() {

    while (true) {
		pthread_mutex_lock(&q_mutex);
		if (q_out.empty()) {
			pthread_mutex_unlock(&q_mutex);
			return;
		}
		buff_data command = q_out.front();
		q_out.pop();
		pthread_mutex_unlock(&q_mutex);

      switch (command.action_type)
      {
      case MOVE:
        sendMove(command.joint, command.speed, command.position);
        break;
      case UPDATE_JOINTS:
        pthread_mutex_lock(&read_servo_mutex);
        open_servo.updateBasicValuesAllServo();
        pthread_mutex_unlock(&read_servo_mutex);
        break;
      }
    }
}
*/

void OpenServoRobot::threadRutine()
{
  cout << "** Thread rutine: IO openservo" << endl;
  //sv* servo;
  int action_type;
  buff_data tmp_data;
  
  static uint32_t count1 = 0;
  static uint32_t count2 = 0;
  static uint32_t count3 = 0;
  
  bool end_loop = false;
  
  //while(!end_loop)
  while (true)
  {
    count3++;
    //cout << "** IO thread loop\n";
    ///pthread_mutex_lock(&q_mutex);
    ///bool empty = q_out.empty();
    ///pthread_mutex_unlock(&q_mutex);
    
    //while(!empty)
    while (true)
    {
      pthread_mutex_lock(&q_mutex);
      if (q_out.empty()) {
		//cout << "q_out is empty!" << flush;
        pthread_mutex_unlock(&q_mutex);
        break;
      }
      //cout << "q_out is not empty!" << flush;
      action_type = q_out.front().action_type;
      tmp_data = q_out.front();
      q_out.pop();
      pthread_mutex_unlock(&q_mutex);
      //pthread_mutex_unlock(&q_mutex);
      //switch(action_type)
      ///cout << "IO TH: switch... " << flush;
      switch(tmp_data.action_type)
      {
        case MOVE:
          //pthread_mutex_lock(&q_mutex);
          //tmp_data = q_out.front();
          //q_out.pop();
          //pthread_mutex_unlock(&q_mutex);
          //cout << "** Thread IO, MOVE joint: " << tmp_data.joint << endl;
          sendMove(tmp_data.joint, tmp_data.speed, tmp_data.position);
          count1++;
        break;
        case UPDATE_JOINTS:
          //pthread_mutex_lock(&q_mutex);
          //q_out.pop();
          //pthread_mutex_unlock(&q_mutex);
          pthread_mutex_lock(&read_servo_mutex);
          open_servo.updateBasicValuesAllServo();
          pthread_mutex_unlock(&read_servo_mutex);
          //cout << "update joints\n";
          count2++;
        break;
      }
      //cout << "q_out6" << endl;
      ///cout << " END" << endl;
      
      //pthread_mutex_lock(&q_mutex);
      //empty = q_out.empty();
      //pthread_mutex_unlock(&q_mutex);
    }
    
    // varneje bi blo prej še poslati/počakati oz. sprazniti vrsto
    if (end_loop)
      break;
    
    //cout << "** IO thread loop -> sleep\n";
    // go to sleep
    pthread_mutex_lock(&sleep_mutex);
    in_slipe = true;
    pthread_mutex_unlock(&sleep_mutex);
    
    //cout << "** IO thread loop -> wait for signal\n";
    // waiting for signal
    pthread_mutex_lock(&sleep_mutex); // potrebno?
    //cout << "** IO thread loop -> wait for signal1\n";
    pthread_cond_wait(&wake_up_condition, &sleep_mutex);
    //cout << "** IO thread loop -> wait for signal2\n";
    end_loop = end_thread;
    //cout << "** IO thread loop -> wait for signal3\n";
    pthread_mutex_unlock(&sleep_mutex);
    //cout << "** IO thread loop -> wait for signal4\n";
    
    //cout << "** IO thread loop -> go out of sleep\n";
    pthread_mutex_lock(&sleep_mutex);
    in_slipe = false;
    pthread_mutex_unlock(&sleep_mutex);
    // tudi če je end_loop true moramo prej sprazniti vrsto
    //cout << "\rThread routine loop = " << count3 << " : Move = " << count1 << " Update = " << count2 << flush;
  }
  
  cout << "\n** Thread routine: IO openservo -> END" << endl;
  // je potrebno sprostiti še kakšne vire?
  pthread_exit(NULL);
}

void OpenServoRobot::threadRutineReq()
{
  cout << "** Thread routine: Request" << endl;
  //static uint32_t count = 0;
  int time_diff = 0;
  chrono::steady_clock::time_point begin;
  //chrono::microseconds interval_micro(1000000/read_rate);
  
  chrono::microseconds interval_micro(8000000/read_rate);
  
  //cout << "request, micro: " << chrono::duration_cast<chrono::microseconds>(interval_micro).count() << "us" << endl;
  bool end_loop = false;
  //auto zamik = chrono::duration_cast<chrono::microseconds>(begin - chrono::steady_clock::now()).count();
  begin = chrono::steady_clock::now();
  while(true)
  {
    //cout << " Req. thread loop -> request\n";
    
    updateJoints();
    
    //cout << " Req. thread loop -> request in que\n";
    
    pthread_mutex_lock(&sleep_mutex);
    end_loop = end_thread;
    pthread_mutex_unlock(&sleep_mutex);
    
    //cout << "end loop: " << end_loop << endl;
    if (end_loop)
      break;

    //cout << " Req. thread loop -> sleep\n";
    //cout <<"\rReq. thread loop -> " << count++ << flush; // end at 1460
    begin += interval_micro;
    //usleep(chrono::duration_cast<chrono::microseconds>(begin - chrono::steady_clock::now()).count());
    time_diff = chrono::duration_cast<chrono::microseconds>(begin - chrono::steady_clock::now()).count();
    if (time_diff > 0)
        usleep(time_diff);

    //else
    //  cout << "  Req. thread loop: time_diff = " << time_diff << endl;
  }
  
  cout << "\n** Thread routine: Request -> END" << endl;
  // je potrebno sprostiti še kakšne vire?
  pthread_exit(NULL);
}

void OpenServoRobot::updateJoints()
{
  bool tmp_in_slip;
  
  buff_data tmp;
  tmp.action_type = UPDATE_JOINTS;
  
  pthread_mutex_lock (&q_mutex);
  //cout << q_out << endl;
  q_out.push(tmp);
  pthread_mutex_unlock (&q_mutex);
  
  pthread_mutex_lock(&sleep_mutex);
  tmp_in_slip = in_slipe;
  pthread_mutex_unlock(&sleep_mutex);
  
  if (tmp_in_slip)
    pthread_cond_signal(&wake_up_condition);
}

