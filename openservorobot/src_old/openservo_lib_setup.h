#ifndef __OPENSERVO_LIB_SETUP
#define __OPENSERVO_LIB_SETUP

// openservo configuration options
#define MAX_INTERFACES       64 
#define I2C_DEVICE_OPENSERVO 1           //device type
#define I2C_MAX_DATA_LEN     32
#define MAX_I2C_DEVICES      127         //number of devices on a bus

/*
#define DEADBAND             1 // sets the width of the deadband
#define TWI_ADDRESS          2
#define PID_PGAIN            3
#define PID_DGAIN            4
#define PID_IGAIN            5
#define MIN_SEEK             6
#define MAX_SEEK             7
*/

//** register addresses
// TWI read/only status registers.  Writing
// values to these registers has no effect.
#define DEVICE_TYPE             0x00
#define DEVICE_SUBTYPE          0x01

#define VERSION_MAJOR           0x02
#define VERSION_MINOR           0x03

#define FLAGS_HI                0x04
#define FLAGS_LO                0x05

#define TIMER_HI                0x06
#define TIMER_LO                0x07

#define POSITION_HI             0x08 // Read Only Servo position high byte
#define POSITION_LO             0x09 // Read Only Servo position low byte

#define VELOCITY_HI             0x0A // Read Only Servo velocity high byte
#define VELOCITY_LO             0x0B // Read Only Servo velocity low byte

#define POWER_HI                0x0C // Read Only Servo power high byte
#define POWER_LO                0x0D // Read Only Servo power low byte

#define PWM_CW                  0x0E // Read Only PWM clockwise value
#define PWM_CCW                 0x0F // Read Only PWM counter-clockwise value


//** rw addresses
// TWI read/write registers.  Writing these
// registers controls operation of the servo.
#define SEEK_HI                 0x10 // Read/Write Seek position high byte
#define SEEK_LO                 0x11 // Read/Write Seek position low byte

#define SEEK_VELOCITY_HI        0x12
#define SEEK_VELOCITY_LO        0x13

#define VOLTAGE_HI              0x14
#define VOLTAGE_LO              0x15

#define CURVE_RESERVED          0x16
#define CURVE_BUFFER            0x17

#define CURVE_DELTA_HI          0x18
#define CURVE_DELTA_LO          0x19

#define CURVE_POSITION_HI       0x1A
#define CURVE_POSITION_LO       0x1B

#define CURVE_IN_VELOCITY_HI    0x1C
#define CURVE_IN_VELOCITY_LO    0x1D

#define CURVE_OUT_VELOCITY_HI   0x1E
#define CURVE_OUT_VELOCITY_LO   0x1F


//** rw protected addresses
// TWI safe read/write registers.  These registers
// may only be written to when write enabled.
#define TWI_ADDRESS             0x20

#define PID_DEADBAND            0x21

#define PID_PGAIN_HI            0x22 // PID proportional gain high byte
#define PID_PGAIN_LO            0x23 // PID proportional gain low byte

#define PID_DGAIN_HI            0x24 // PID derivative gain high byte
#define PID_DGAIN_LO            0x25 // PID derivative gain low byte

#define PID_IGAIN_HI            0x26 // PID integral gain high byte
#define PID_IGAIN_LO            0x27 // PID integral gain low byte

#define PWM_FREQ_DIVIDER_HI     0x28
#define PWM_FREQ_DIVIDER_LO     0x29

#define MIN_SEEK_HI             0x2A // Minimum seek position high byte
#define MIN_SEEK_LO             0x2B // Minimum seek position low byte

#define MAX_SEEK_HI             0x2C // Maximum seek position high byte
#define MAX_SEEK_LO             0x2D // Maximum seek position low byte

#define REVERSE_SEEK            0x2E // Reverse seek sense

#define RESERVED_2F             0x2F

#define SERVO_ID_HI             0x30  // servo ID value
#define SERVO_ID_LO             0x31
#define CURRENT_CUT_OFF_HI      0x32  // current cut off value, with delay
#define CURRENT_CUT_OFF_LO      0x33
#define CURRENT_SOFT_CUT_OFF_HI 0x34  // current cut off value
#define CURRENT_SOFT_CUT_OFF_LO 0x35

// naslovi od 0x2F do 0x80 še niso porabljeni, kar znese 80B


//** command addresses
#define RESET                   0x80 // Reset microcontroller
#define CHECKED_TXN             0x81 // Read/Write registers with simple checksum

#define PWM_ENABLE              0x82 // Enable PWM to motors
#define PWM_DISABLE             0x83 // Disable PWM to servo motors

#define WRITE_ENABLE            0x84 // Enable write of read/write protected registers
#define WRITE_DISABLE           0x85 // Disable write of read/write protected registers

#define REGISTERS_SAVE          0x86 // Save read/write protected registers fo EEPROM
#define REGISTERS_RESTORE       0x87 // Restore read/write protected registers from EEPROM
#define REGISTERS_DEFAULT       0x88 // Restore read/write protected registers to defaults

#define OS_RESET                0x80 // Reset the servo

// Define the flag register REG_FLAGS_HI and REG_FLAGS_LO bits.
//

#define FLAGS_HI_RESERVED_07        0x07
#define FLAGS_HI_RESERVED_06        0x06
#define FLAGS_HI_RESERVED_05        0x05
#define FLAGS_HI_RESERVED_04        0x04
#define FLAGS_HI_RESERVED_03        0x03
#define FLAGS_HI_RESERVED_02        0x02
#define FLAGS_HI_RESERVED_01        0x01
#define FLAGS_HI_RESERVED_00        0x00

#define FLAGS_LO_RESERVED_07        0x07
#define FLAGS_LO_CURRENT_STATE_1    0x06  // current state bit 1
#define FLAGS_LO_CURRENT_STATE_0    0x05  // current state bit 0
#define FLAGS_LO_MOVING_STATE_1     0x04  // moving state bit 1
#define FLAGS_LO_MOVING_STATE_0     0x03  // moving state bit 0
#define FLAGS_LO_MOTION_ENABLED     0x02
#define FLAGS_LO_WRITE_ENABLED      0x01
#define FLAGS_LO_PWM_ENABLED        0x00

#define CF_REVERSE_SEEK     1  // Reverse seek default value
#define CF_READ_DIRECT      1  // Reverse seek default value

#endif
