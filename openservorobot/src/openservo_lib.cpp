#include "openservo_lib.h"
#include <unistd.h>

#define NON_BLOCKING_RETURN_VALUE 1;

//**********************************************************************
// JAVNE METODE

OpenServo::OpenServo()
{
  i2c_file = 0;
  i2c_file_open = 0;
}

OpenServo::~OpenServo()
{
  closePort();
}


int OpenServo::openPort(string port)
{
  if (i2cOpen(&port[0], &i2c_file) != 1)
    return -1;
  i2c_file_open = 1;
  port_name = port;
  return 1;
}

int OpenServo::closePort()
{
  i2c_file_open = 0;
  return i2cClose(i2c_file);
}

int OpenServo::scanPort(unsigned char *addr)
{
  if (!i2c_file_open)
    return -1;
  return i2cScann(i2c_file, addr);
}

//
int OpenServo::scanPortAutoAddServo()
{
  int added = autoAddServo();
  
  // podaj zahtevo za celotno posodobitev vseh podatkov vseh dodanih servomotorjev
  updateServoValuesAllServo();
  return added;
}

// posodobi celoten seznam servo motorjev
int OpenServo::refreshServoList()
{
  autoRemoveServo();
  autoAddServo();
  return servos.size();
}

// Zahtevamo, da se podatki na naši strani posodobijo s podatki ki so dejansko v servo motorju
int OpenServo::updateServoValues(int servo_id)
{
  // preveri ali servo_id obstaja v našem seznamu, če ne return -1;
  sv* servo;
  if ((servo=findServo(servo_id)) == NULL)
    return -1;
  
  int data_length = CURRENT_SOFT_CUT_OFF_LO+1; // želimo prebrati od 1. do registra na tem naslovu, kar tudi podaja št bajtov
  unsigned char data[data_length];
  
  if (readData((unsigned char)servo->address, 0x00, data, data_length) != 1)
    return -1;
  
  // posodabljanje
  updateServoLocalAll(servo, data);
  return 1;
}

int OpenServo::updateServoValuesAllServo()
{
  int stat = 0;
  for (vector<sv>::iterator it=servos.begin(); it!=servos.end(); it++)
  {
    // TODO: FIXIT
    //stat += updateServoValues(it->type);
    stat += updateServoValues(it->address);
    
  }
  return stat;
}

// posodobijo se:
//   - zastavice
//   - trenutna pozicija
//   - iskana pozicija
//   - trenutna hitrost
//   - iskana hitrost
//   - tok
//   - pwm cw in ccw
int OpenServo::updateBasicValues(int servo_id)
{
  // preveri ali servo_id obstaja v našem seznamu, če ne return -1;
  sv* servo;
  if ((servo=findServo(servo_id)) == NULL)
    return -1;
  
  int data_length = VOLTAGE_LO - FLAGS_HI +1; // okvirno 18B
  unsigned char data[data_length];
  
  if (readData((unsigned char)servo->address, FLAGS_HI, data, data_length) != 1)
    return -1;
  
  // posodabljanje
  updateServoLocalBasic(servo, data, FLAGS_HI);
  return 1;
}

int OpenServo::updateBasicValuesAllServo()
{
  int stat = 0;
  for (vector<sv>::iterator it=servos.begin(); it!=servos.end(); it++)
  {
    //stat += updateBasicValues(it->type);
    stat += updateBasicValues(it->address);
  }
  return stat;
}

int OpenServo::resetServo(int servo_id)
{
  return command1B(servo_id, RESET);
}

int OpenServo::enableMotor(int servo_id)
{
  return command1B(servo_id, PWM_ENABLE);
}

int OpenServo::disableMotor(int servo_id)
{
  return command1B(servo_id, PWM_DISABLE);
}

int OpenServo::writeEnable(int servo_id)
{
  return command1B(servo_id, WRITE_ENABLE);
}

int OpenServo::writeDisable(int servo_id)
{
  return command1B(servo_id, WRITE_DISABLE);
}

int OpenServo::registerSave(int servo_id)
{
  return command1B(servo_id, REGISTERS_SAVE);
}

int OpenServo::registerRestore(int servo_id)
{
  return command1B(servo_id, REGISTERS_RESTORE);
}

int OpenServo::registerDeafult(int servo_id)
{
  return command1B(servo_id, REGISTERS_DEFAULT);
}


// nastavi parametre
int OpenServo::setSeekPossition(int servo_id, int value)
{
  return write2B(servo_id, SEEK_HI, value);
}

int OpenServo::setSeekVelocity(int servo_id, int value)
{
  return write2B(servo_id, SEEK_VELOCITY_HI, value);
}

int OpenServo::setCurveInVelocity(int servo_id, int value)
{
  return write2B(servo_id, CURVE_IN_VELOCITY_HI, value);
}

int OpenServo::setCurveOutVelocity(int servo_id, int value)
{
  return write2B(servo_id, CURVE_OUT_VELOCITY_HI, value);
}

int OpenServo::setAddress(int servo_id, int value)
{
  return write1B(servo_id, TWI_ADDRESS, value);
}

int OpenServo::setPidDeadband(int servo_id, int value)
{
  return write1B(servo_id, PID_DEADBAND, value);
}

int OpenServo::setPidGain(int servo_id, int proportional, int derivative, int integral)
{
  return 0;
}

int OpenServo::setPidProportionalGain(int servo_id, int value)
{
  return write2B(servo_id, PID_PGAIN_HI, value);
}

int OpenServo::setPidDerivativeGain(int servo_id, int value)
{
  return write2B(servo_id, PID_DGAIN_HI, value);
}

int OpenServo::setPidIntegralGain(int servo_id, int value)
{
  return write2B(servo_id, PID_IGAIN_HI, value);
}

int OpenServo::setPwmFreqDivider(int servo_id, int value)
{
  return write2B(servo_id, PWM_FREQ_DIVIDER_HI, value);
}

int OpenServo::setMinSeekToCurrentPos(int servo_id)
{
  return 0;
}

int OpenServo::setMinSeek(int servo_id, int value)
{
  return write2B(servo_id, MIN_SEEK_HI, value);
}

int OpenServo::setMaxSeekToCurrentPos(int servo_id)
{
  return 0;
}

int OpenServo::setMaxSeek(int servo_id, int value)
{
  return write2B(servo_id, MAX_SEEK_HI, value);
}

int OpenServo::setReverseSeek(int servo_id)
{
  return 0;
}

int OpenServo::setReverseSeekTo(int servo_id, int value)
{
  return write1B(servo_id, REVERSE_SEEK, value);
}

int OpenServo::setServoID(int servo_id, int value)
{
  return write2B(servo_id, SERVO_ID_HI, value);
}

int OpenServo::setCurrentCutOff(int servo_id, int value)
{
  return write2B(servo_id, CURRENT_CUT_OFF_HI, value);
}

int OpenServo::setCurrentSoftCuttOff(int servo_id, int value)
{
  return write2B(servo_id, CURRENT_SOFT_CUT_OFF_HI, value);
}

sv OpenServo::getServo(int servo_id)
{
  return *findServo(servo_id);
}

int OpenServo::getNumOfServos()
{
  return servos.size();
}

//**********************************************************************
// PRIVATNE METODE:

void OpenServo::defaultParameters()
{
  
}

// iz seznama odstrani servomotorje, ki niso več priklopljeni na vodilo i2c
int OpenServo::autoRemoveServo()
{
  if(!i2c_file_open)
    return -1;
  
  unsigned char tmp[128];
  int count = i2cScann(i2c_file, tmp);
  
  int counter = 0;
  
  // odstrani neobstoječ servo
  if (servos.size() > 0)
  {
    if (count == 0)
    {
      counter = servos.size();
      servos.clear();
      return counter;
    }
    for (int w=0; w<count; w++)
    {
      for (int q=0; q<servos.size(); q++)
      {
        if (servos[q].address = (unsigned int)tmp[w])
        {
          servos.erase(servos.begin()+q);
          counter++;
          break;
        }
      }
    }
    
  }
  return counter;
}

// v seznam doda trenutno nedodane servomotorje
int OpenServo::autoAddServo()
{
  if(!i2c_file_open)
    return -1;
  
  unsigned char tmp[128];
  int count = i2cScann(i2c_file, tmp);
  //cout << "\t- count = " << count << endl;
  int counter = 0;
  
  // dodaj nov servo
  for (int q=0; q<count; q++)
  {
    if (!servoInVector((unsigned int)tmp[q]))
    {
      sv tmp_sv;
      tmp_sv.address = (unsigned int)tmp[q];      
      servos.push_back(tmp_sv);
      counter++;
    }
  }
  //cout << "\t- counter = " << counter << endl;
  return counter;
}

// preveri, ali v vektorju že obstaja servo s podanim naslovom
bool OpenServo::servoInVector(int servo_addr)
{
  for (int q=0; q<servos.size(); q++)
  {
    if (servos[q].address == servo_addr)
      return true;
  }
  
  return false;
}

// pošiljanje enega ukaza (redudatno?)
int OpenServo::sendCommand(int addr, unsigned char cmd)
{
  if (i2cSetSlave(i2c_file, addr) != 1)
    return -1;
  
  // 7. bit pove, ali gre za ukaz(1) ali naslov (0)
  //cmd |= 1<<8; // 
  cmd |= 0x80;
  return i2cWrite(i2c_file, &cmd, 1);
  
  //return 1;
}

// pošiljanje več ukazov servo motorju
int OpenServo::sendCommand(unsigned char addr, unsigned char* cmd, int len)
{
  if (i2cSetSlave(i2c_file, addr) != 1)
    return -1;
  
  // 7. bit pove, ali gre za ukaz(1) ali naslov (0)
  for (int q=0; q<len; q++)
    //cmd[q] |= 1<<8; // 
    cmd[q] |= 0x80;
  return i2cWrite(i2c_file, cmd, len);
}

// pošiljanje podatkov na servo motor
int OpenServo::sendData(unsigned char addr, unsigned char data_addr, unsigned char* data, int data_len)
{
  if (i2cSetSlave(i2c_file, addr) != 1)
    return -1;
  
  // 7. bit pove, ali gre za ukaz(1) ali naslov (0)
  data_addr &= 0x7F;
  unsigned char tmp_ch[data_len+1];
  tmp_ch[0] = data_addr;
  if (data_len > 0)
    memcpy(&tmp_ch[1], data, data_len);
  return i2cWrite(i2c_file, tmp_ch, data_len+1);
}

// branje s servo motorja
int OpenServo::readData(unsigned char addr, unsigned char data_addr, unsigned char* data, int data_len)
{
  if (i2cSetSlave(i2c_file, addr) != 1)
    return -1;
  
  //najprej zapišemo/pošljemo naslov, s katerega želimo pričeti z branjem
  // 7. bit pove, ali gre za ukaz(1) ali naslov (0)
  data_addr &= 0x7F;
  if (i2cWrite(i2c_file, &data_addr, 1) != 1)
    return -1;
    
  // nato pričnemo z branjem želenega števila bajtov
  return i2cRead(i2c_file, data, data_len);
  
}

// na naši strani posodobimo posamezne parametre servo motorja
void OpenServo::updateServoLocal(sv* servo_up, int cmd, unsigned char* data)
{
  // nastavi zastavico, da je prišlo do spremembe podatkov!
  int change = 1;
  switch(cmd)
  {
    case DEVICE_TYPE:
      servo_up->type = (int)data[0];
      break;
    case DEVICE_SUBTYPE:
      servo_up->subtype = (int)data[0];
      break;
    case VERSION_MAJOR:
      servo_up->version = ((int)data[0]) << 8;
      servo_up->version |= (int)data[1];
      break;
    case FLAGS_HI:
      servo_up->flags = ((int)data[0]) << 8;
      servo_up->flags |= (int)data[1];
      break;
    case TIMER_HI:
      servo_up->timer = ((int)data[0]) << 8;
      servo_up->timer |= (int)data[1];
      break;
    case POSITION_HI:
      servo_up->position = ((int)data[0]) << 8;
      servo_up->position |= (int)data[1];
      break;
    case VELOCITY_HI:
      servo_up->velocity = ((int)data[0]) << 8;
      servo_up->velocity |= (int)data[1];
      break;
    case POWER_HI:
      servo_up->current = ((int)data[0]) << 8;
      servo_up->current |= (int)data[1];
      break;
    case PWM_CW:
      servo_up->pwm_cw = (int)data[0];
      break;
    case PWM_CCW:
      servo_up->pwm_ccw = (int)data[0];
      break;
    case SEEK_HI:
      servo_up->seek_position = ((int)data[0]) << 8;
      servo_up->seek_position |= (int)data[1];
      break;
    case SEEK_VELOCITY_HI:
      servo_up->seek_velocity = ((int)data[0]) << 8;
      servo_up->seek_velocity |= (int)data[1];
      break;
    case VOLTAGE_HI:
      servo_up->voltage = ((int)data[0]) << 8;
      servo_up->voltage |= (int)data[1];
      break;
    case CURVE_DELTA_HI:
      servo_up->curve_delta = ((int)data[0]) << 8;
      servo_up->curve_delta |= (int)data[1];
      break;
    case CURVE_POSITION_HI:
      servo_up->curve_position = ((int)data[0]) << 8;
      servo_up->curve_position |= (int)data[1];
      break;
    case CURVE_IN_VELOCITY_HI:
      servo_up->curve_in_velocity = ((int)data[0]) << 8;
      servo_up->curve_in_velocity |= (int)data[1];
      break;
    case CURVE_OUT_VELOCITY_HI:
      servo_up->curve_out_velocity = ((int)data[0]) << 8;
      servo_up->curve_out_velocity |= (int)data[1];
      break;
    case TWI_ADDRESS:
      servo_up->address = (int)data[0];
      break;
    case PID_DEADBAND:
      servo_up->deadband = (int)data[0];
      break;
    case PID_PGAIN_HI:
      servo_up->pgain = ((int)data[0]) << 8;
      servo_up->pgain |= (int)data[1];
      break;
    case PID_DGAIN_HI:
      servo_up->dgain = ((int)data[0]) << 8;
      servo_up->dgain |= (int)data[1];
      break;
    case PID_IGAIN_HI:
      servo_up->igain = ((int)data[0]) << 8;
      servo_up->igain |= (int)data[1];
      break;
    case PWM_FREQ_DIVIDER_HI:
      servo_up->pwm_freq_divider = ((int)data[0]) << 8;
      servo_up->pwm_freq_divider |= (int)data[1];
      break;
    case MIN_SEEK_HI:
      servo_up->minseek = ((int)data[0]) << 8;
      servo_up->minseek |= (int)data[1];
      break;
    case MAX_SEEK_HI:
      servo_up->maxseek = ((int)data[0]) << 8;
      servo_up->maxseek |= (int)data[1];
      break;
    case REVERSE_SEEK:
      servo_up->reverse_seek = (int)data[0];
      break;
    case SERVO_ID_HI:
      servo_up->maxseek = ((int)data[0]) << 8;
      servo_up->maxseek |= (int)data[1];
      break;
    case CURRENT_CUT_OFF_HI:
      servo_up->maxseek = ((int)data[0]) << 8;
      servo_up->maxseek |= (int)data[1];
      break;
    case CURRENT_SOFT_CUT_OFF_HI:
      servo_up->maxseek = ((int)data[0]) << 8;
      servo_up->maxseek |= (int)data[1];
      break;
    default:
      change = 0;
  }
  
  if (change && !servo_up->updated)
  {
    servo_up->updated = true;
  }
  
}

void OpenServo::updateServoLocalAll(sv* servo_up, unsigned char* data)
{
  servo_up->type =                (int)data[DEVICE_TYPE];
  servo_up->subtype =             (int)data[DEVICE_SUBTYPE];
  servo_up->version =             ((int)data[VERSION_MAJOR]) << 8;
  servo_up->version |=            (int)data[VERSION_MINOR];
  servo_up->flags =               ((int)data[FLAGS_HI]) << 8;
  servo_up->flags |=              (int)data[FLAGS_LO];
  servo_up->timer =               ((int)data[TIMER_HI]) << 8;
  servo_up->timer |=              (int)data[TIMER_LO];
  servo_up->position =            ((int)data[POSITION_HI]) << 8;
  servo_up->position |=           (int)data[POSITION_LO];
  servo_up->velocity =            (short int)(data[VELOCITY_HI] << 8);
  servo_up->velocity |=           (int)data[VELOCITY_LO];
  servo_up->current =             ((int)data[POWER_HI]) << 8;
  servo_up->current |=            (int)data[POWER_LO];
  servo_up->pwm_cw =              (int)data[PWM_CW];
  servo_up->pwm_ccw =             (int)data[PWM_CCW];
  servo_up->seek_position =       ((int)data[SEEK_HI]) << 8;
  servo_up->seek_position |=      (int)data[SEEK_LO];
  servo_up->seek_velocity =       ((int)data[SEEK_VELOCITY_HI]) << 8;
  servo_up->seek_velocity |=      (int)data[SEEK_VELOCITY_LO];
  servo_up->voltage =             ((int)data[VOLTAGE_HI]) << 8;
  servo_up->voltage |=            (int)data[VOLTAGE_LO];
  servo_up->curve_delta =         ((int)data[CURVE_DELTA_HI]) << 8;
  servo_up->curve_delta |=        (int)data[CURVE_DELTA_LO];
  servo_up->curve_position =      ((int)data[CURVE_POSITION_HI]) << 8;
  servo_up->curve_position |=     (int)data[CURVE_POSITION_LO];
  servo_up->curve_in_velocity =   ((int)data[CURVE_IN_VELOCITY_HI]) << 8;
  servo_up->curve_in_velocity |=  (int)data[CURVE_IN_VELOCITY_LO];
  servo_up->curve_out_velocity =  ((int)data[CURVE_OUT_VELOCITY_HI]) << 8;
  servo_up->curve_out_velocity |= (int)data[CURVE_OUT_VELOCITY_LO];
  servo_up->address =             (int)data[TWI_ADDRESS];
  servo_up->deadband =            (int)data[PID_DEADBAND];
  servo_up->pgain =               ((int)data[PID_PGAIN_HI]) << 8;
  servo_up->pgain |=              (int)data[PID_PGAIN_LO];
  servo_up->dgain =               ((int)data[PID_DGAIN_HI]) << 8;
  servo_up->dgain |=              (int)data[PID_DGAIN_LO];
  servo_up->igain =               ((int)data[PID_IGAIN_HI]) << 8;
  servo_up->igain |=              (int)data[PID_IGAIN_LO];
  servo_up->pwm_freq_divider =    ((int)data[PWM_FREQ_DIVIDER_HI]) << 8;
  servo_up->pwm_freq_divider |=   (int)data[PWM_FREQ_DIVIDER_LO];
  servo_up->minseek =             ((int)data[MIN_SEEK_HI]) << 8;
  servo_up->minseek |=            (int)data[MIN_SEEK_LO];
  servo_up->maxseek =             ((int)data[MAX_SEEK_HI]) << 8;
  servo_up->maxseek |=            (int)data[MAX_SEEK_LO];
  servo_up->reverse_seek =        (int)data[REVERSE_SEEK];
  servo_up->id =                  ((int)data[SERVO_ID_HI]) << 8;
  servo_up->id |=                 (int)data[SERVO_ID_LO];
  servo_up->current_cut_off =     ((int)data[CURRENT_CUT_OFF_HI]) << 8;
  servo_up->current_cut_off |=    (int)data[CURRENT_CUT_OFF_LO];
  servo_up->current_soft_cut_off = ((int)data[CURRENT_SOFT_CUT_OFF_HI]) << 8;
  servo_up->current_soft_cut_off |= (int)data[CURRENT_SOFT_CUT_OFF_LO];
  servo_up->updated = true;
}

void OpenServo::updateServoLocalBasic(sv* servo_up, unsigned char* data, int offset)
{
  servo_up->flags = ((int)data[FLAGS_HI-offset]) << 8;
  servo_up->flags |= (int)data[FLAGS_LO-offset];
  servo_up->timer = ((int)data[TIMER_HI-offset]) << 8;
  servo_up->timer |= (int)data[TIMER_LO-offset];
  servo_up->position = ((int)data[POSITION_HI-offset]) << 8;
  servo_up->position |= (int)data[POSITION_LO-offset];
  servo_up->velocity = (short int)(data[VELOCITY_HI-offset] << 8);
  servo_up->velocity |= (int)data[VELOCITY_LO-offset];
  servo_up->current = ((int)data[POWER_HI-offset]) << 8;
  servo_up->current |= (int)data[POWER_LO-offset];
  servo_up->pwm_cw = (int)data[PWM_CW-offset];
  servo_up->pwm_ccw = (int)data[PWM_CCW-offset];
  servo_up->seek_position = ((int)data[SEEK_HI-offset]) << 8;
  servo_up->seek_position |= (int)data[SEEK_LO-offset];
  servo_up->seek_velocity = ((int)data[SEEK_VELOCITY_HI-offset]) << 8;
  servo_up->seek_velocity |= (int)data[SEEK_VELOCITY_LO-offset];
  servo_up->voltage = ((int)data[VOLTAGE_HI-offset]) << 8;
  servo_up->voltage |= (int)data[VOLTAGE_LO-offset];

  servo_up->updated = true;
}

int OpenServo::findServo(sv* servo, int servo_id)
{
  for (int q=0; q<servos.size(); q++)
  {
    if (servos[q].type == servo_id)
    {
      servo = &servos[q];
      return 1;
    }
  }
  return 0;
}

sv* OpenServo::findServo(int servo_id)
{
  //return &servos[0];
  for (int q=0; q<servos.size(); q++)
  {
    //if (servos[q].type == servo_id)
    if (servos[q].address == servo_id) // le začasna rešitev, dokler se ne definira tipov...
      return &servos[q];
  }
  return NULL;
}

int OpenServo::write2B(int servo_id, unsigned char data_addr, int value)
{
  // preveri ali servo_id obstaja v našem seznamu, če ne return -1;
  sv* servo;
  if ((servo=findServo(servo_id)) == NULL)
    return -1;
  
  unsigned char data[2];
  data[1] = (unsigned char)value;
  data[0] = (unsigned char)(value >> 8);
  //data[1] = (unsigned char)(value%0xFF);
  
  //cout << value << " " << (unsigned int) data[0] << " " << (unsigned int) data[1] << endl; ; 
  
  return sendData(servo->address, data_addr, data, 2);
}

int OpenServo::write1B(int servo_id, unsigned char data_addr, int value)
{
  // preveri ali servo_id obstaja v našem seznamu, če ne return -1;
  sv* servo;
  if ((servo=findServo(servo_id)) == NULL)
    return -1;
  
  unsigned char data[1];
  data[0] = (unsigned char)value;
  
  return sendData(servo->address, data_addr, data, 1);
}

int OpenServo::command1B(int servo_id, unsigned char cmd)
{
  // preveri ali servo_id obstaja v našem seznamu, če ne return -1;
  sv* servo;
  if ((servo=findServo(servo_id)) == NULL)
    return -1;
  
  return sendCommand(servo->address, cmd);
}




//----------------------------------------------------------------------

void OpenServo::correctAddress()
{
  unsigned char adr[128];
  int num = scanPort(adr);
  int sl = 100;
  cout << "correct adrs:"<<endl;
  cout << "  num:" << num <<endl;
  for (int q=0; q<num; q++)
  {
    cout << "adr: " << (int)adr[q] << endl;
    cout << "  - " << sendCommand(adr[q], WRITE_ENABLE) << endl;
    usleep(sl);
    cout << "  - " << sendData(adr[q], TWI_ADDRESS, &adr[q], 1) << endl;
    usleep(sl);
    cout << "  - " << sendCommand(adr[q], REGISTERS_SAVE) << endl;
    usleep(sl);
    cout << "  - " << sendCommand(adr[q], WRITE_DISABLE) << endl;
    usleep(sl);
    cout << "  - " << sendCommand(adr[q], RESET) << endl;
    usleep(10*sl);
  }
  cout << endl;
}
