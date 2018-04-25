
//fix for g++ compilers
#ifdef __cplusplus 
extern "C" {
#endif

//#ifdef LIB_I2C_H
//#define LIB_I2C_H

int i2cOpen (char *filename, int *file);
int i2cClose(int file);
int i2cSetSlave(int file, int i2c_dev_addr);
int i2cRead(int file, unsigned char* buffer, int length);
int i2cWrite(int file, unsigned char* buffer, int length);
int i2cScann(int file, unsigned char* addr);


//#endif

//fix for g++ compilers
#ifdef __cplusplus 
}
#endif

/*
class libi2c
{
public:
	libi2c();
	~libi2c();
	
	Open(char *filename);
	Close();
	SetSlaveAddr(int addr);
	Read();
	Write();
	
private:
	int file_i2c;
	int file_i2c_open = 0;
	int length;
	unsigned char buffer[60] = {0};
}
*/
