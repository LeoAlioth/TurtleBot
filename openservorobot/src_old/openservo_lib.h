#ifndef __OPENSERVO_LIB
#define __OPENSERVO_LIB

#include <string>
#include <iostream>
#include <vector>
#include <cstring>
//#include <queue>
//#include <pthread.h>
//#include <functional>

#include "i2c.h"
#include "openservo_lib_setup.h"

using namespace std;


//typedef 
struct sv
{
  bool responsive;  // TODO: se še ne nastavlja!!
  bool updated;
  // read only (11 values)
  int type;                // OpenServo device type
  int subtype;             // OpenServo device subtype
  int version;             // version number of OpenServo software
  int flags;
  int timer;               // Timer -­ incremented each ADC sample
  int position;            // Servo position
  int velocity;            // Servo velocity
  int current;             // Servo current/power
  int pwm_cw;              // PWM clockwise value
  int pwm_ccw;             // PWM counter­clockwise value
  //int speed;
  
  // read/write (7 values)
  int seek_position;       // Seek position
  int seek_velocity;       // Speed seek position
  int voltage;             // Battery/adapter Voltage value
  int curve_delta;         // Curve Time delta
  int curve_position;      // Curve position
  int curve_in_velocity;   // Curve in velocity
  int curve_out_velocity;  // Curve out velocity
  
  // read/write protected (12 values)
  int address;             // TWI address of servo
  int deadband;            // Programmable PID deadband value
  int pgain;               // PID proportional gain
  int dgain;               // PID derivative gain
  int igain;               // PID integral gain
  int pwm_freq_divider;    // PWM frequency divider
  int minseek;             // Minimum seek position
  int maxseek;             // Maximum seek position
  int reverse_seek;        // reverse seek mode
  int id;                  // servo ID
  int current_cut_off;     // current cut off value, with delay
  int current_soft_cut_off;// current cut off value
  
  int read_direct;
  int checked_crc;
  
  //struct sv *next;
};

class OpenServo
{
public:
  
  
  OpenServo();
  ~OpenServo();
  
  int openPort(string port);
  int closePort();
  int scanPort(unsigned char *addr); // skenira port in vrne število naprav na portu ter v addr njihove naslove
  int scanPortAutoAddServo();  //skenira port in avtomatično doda nov servo v vektor;
  int refreshServoList(); // posodobi seznam vseh servo motorjev (doda nove in odstrani neobstoječe)
  int updateServoValues(int servo_id); // na naši strani posodobimo vse podatke servota, podamo id servota
  int updateBasicValues(int servo_id); // na naši strani se posodobijo le najpogosteje spreminjajoče se vrednosti
  int updateServoValuesAllServo();
  int updateBasicValuesAllServo();
  
  // ukazi
  int resetServo(int servo_id);
  int enableMotor(int servo_id);
  int disableMotor(int servo_id);
  int writeEnable(int servo_id);
  int writeDisable(int servo_id);
  int registerSave(int servo_id);
  int registerRestore(int servo_id);
  int registerDeafult(int servo_id);
  
  // nastavi parametre
  int setSeekPossition(int servo_id, int value);
  int setSeekVelocity(int servo_id, int value);
  //int setCurveDelta(int servo_id, int value);
  //int setCurvePossition(int servo_id, int value);
  int setCurveInVelocity(int servo_id, int value);
  int setCurveOutVelocity(int servo_id, int value);
  
  int setAddress(int servo_id, int address); // spremeni naslov servota na vodilu
  int setPidDeadband(int servo_id, int deadband); // 
  int setPidGain(int servo_id, int proportional, int derivative, int integral);
  int setPidProportionalGain(int servo_id, int proportional);
  int setPidDerivativeGain(int servo_id, int derivative);
  int setPidIntegralGain(int servo_id, int integral);
  int setPwmFreqDivider(int servo_id, int value);
  int setMinSeekToCurrentPos(int servo_id);
  int setMinSeek(int servo_id, int value);
  int setMaxSeekToCurrentPos(int servo_id);
  int setMaxSeek(int servo_id, int value);
  int setReverseSeek(int servo_id);
  int setReverseSeekTo(int servo_id, int value);
  int setServoID(int servo_id, int value);
  int setCurrentCutOff(int servo_id, int value);
  int setCurrentSoftCuttOff(int servo_id, int value);
  
  //vector<sv> servos;// vector of servos
  
  sv getServo(int servo_id); 
  int getNumOfServos();
  
  void correctAddress();
  void sendRooCommand(unsigned char addr, unsigned char* cmd, int len);
  void sendRooData(unsigned char addr, unsigned char* data, int len);
  
protected:
  //vector<sv> servos;// vector of servos
  
private:
  int i2c_file;
  int i2c_file_open;
  string port_name;
  
  vector<sv> servos;// vector of servos
  
  void defaultParameters();
  
  bool servoInVector(int servo_addr);
  bool servoExist();
  int findServo(sv* servo, int servo_id);
  sv* findServo(int servo_id);
  int autoRemoveServo();
  int autoAddServo();
  // OpenServo "komunikacija"; tole sodi bolj pod private
  int sendCommand(int addr, unsigned char cmd); //pošlje en ukaz; opcija je tudi sočasno pošiljanje več ukazov
  int sendCommand(unsigned char addr, unsigned char* cmd, int len);
  int sendData(unsigned char addr, unsigned char data_addr, unsigned char* data, int data_len); // pošlje/zapiše podatke napravi
  int readData(unsigned char addr, unsigned char data_addr, unsigned char* data, int data_len); // z naprave s podanega naslov podatko v *data prebere data_len podatkov
  // **** zgornji readData() je implementiran v načinu blokiranja!
  // TODO: implementiraj v neblokiranem načinu, če je sploh možno
  void updateServoLocal(sv* servo_up, int cmd, unsigned char* data);
  void updateServoLocalAll(sv* servo_up, unsigned char* data);
  void updateServoLocalBasic(sv* servo_up, unsigned char* data, int offset);
  int write2B(int servo_id, unsigned char data_addr, int value);
  int write1B(int servo_id, unsigned char data_addr, int value);
  int command1B(int servo_id, unsigned char cmd);
};

#endif
