// Za I2C port
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

// i2c header
#include "i2c.h"

/*
int adapter_nr = 2; / probably dynamically determined
37    char filename[20];
38    
39    snprintf(filename, 19, "/dev/i2c-%d", adapter_nr);
 */

//---- OPEN THE I2C BUS ----
//char *filename = (char*)"/dev/i2c-1";
int i2cOpen (char *filename, int *file)
{
  if ((*file = open(filename, O_RDWR)) < 0)
  {
    // ERROR HANDLING: you can chech errno to see what went wrong
    //cout << "Failed to open the i2c bus" << endl;
    return -1;
  }
  return 1;
}

int i2cClose(int file)
{
  if (!close(file)){
    // ERROR HANDLING
    return -1;
  }
  return 1;
}

//int addr = 0x5a; // I2c address of the slave
int i2cSetSlave(int file, int i2c_dev_addr)
{
  if (ioctl(file, I2C_SLAVE, i2c_dev_addr) < 0)
  {
    //cout << "Failed to acquire buss access and/or talk to slave.\n" << endl;
    // ERROR HANDLING: you can chech error no. to see what went wrong
    return -1;
  }
  //file_i2c_addr = 1;
  return 1;
}

//---- READ BYTES ----
// efektivno je to le branje iz sistemskega bufferja
//int i2cRead(int file, int addr, unsigned char* buffer, int length)
int i2cRead(int file, unsigned char* buffer, int length)
{
  if (read(file, buffer, length) != length) //read() returns the number of bytes actually read, if it doesn't match, then an error occurred (e.g. no response from the device)
  {
    // ERROR HANDLING: i2c transaction fauled
    //cout << "Failed to read from the i2c buss.\n" << endl;
    return -1;
  }
  //printf("Data read: %s\n", buffer);
  return 1;
  
}
/*
//---- READ BYTES IN BLOCKS ----
// ** Bocking implementation!!
int i2cReadBlock(int file, unsigned char i2c_dev_addr, unsigned char cmd_addr, unsigned char* buffer, int length)
{
  // nastavi naslov naprave, s katero želimo komunicirati
  if (i2cSetSlave(file, i2c_dev_addr) != 1)
    return -1;
  
  // pošlji/piši ukaz, ki predstavlja naslov s katerega želimo brati
  if (i2cWrite(file, &cmd_addr, 1) != 1)
    return -1;
  
  // preberi podatke, ki jih pošlje naprava
  if (i2cRead(file, i2c_dev_addr, buffer, length) != 1)
    return -1;
  
  return 1;
}
*/
//---- WRITE BYTES ----
// za pisanje/pošiljanje novih podatkov ali če želimo kaj prebrati iz naprave (pošljemo 
int i2cWrite(int file, unsigned char* buffer, int length)
{
  if (write(file, buffer, length) != length)
  {
    // ERROR HANDLING: i2c transaction failed
    //cout << "Failed to write to the i2c bus.\n" << endl;
    return -1;
  }
  return 1;
}

/*
//---- WRITE BYTES IN BLOCKS ----
int i2cWriteBlock(int file, unsigned char i2c_dev_addr, unsigned char cmd_addr, unsigned char* buffer, int length)
{
  // nastavi naslov naprave, s katero želimo komunicirati
  if (i2cSetSlave(file, i2c_dev_addr) != 1)
    return -1;
    
  unsigned char tmp[length+1];
  tmp[0] = cmd_addr;
  memcpy( &tmp[1], buffer, length);
  
  return 1;
}
*/

//---- SCANN ADDRESSES ----
// ** Blocking implementation!!
// scans from 8 to 119
int i2cScann(int file, unsigned char* addr)
{
  int count = 0;
  unsigned char buff[1];
  int q;
  for (q=8; q<120; q++)
  {
    i2cSetSlave(file, q);
    if (i2cRead(file, buff, 1) == 1)
    {
      addr[count++] = q;
    }
  }
  return count;
}
