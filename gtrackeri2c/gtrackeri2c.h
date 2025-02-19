//************************************** 
//  gtrackeri2c header file
//**************************************

#ifndef _GTRACKERI2C_H
#define _GTRACKERI2C_H

//*****************************************************
// If you change the value of this define you must also
// change the value of the same define in gmessage.h!!!
//*****************************************************
#define GTRACKER_PERIPHERAL_I2C_ADDR 0x08
//*****************************************************

//#define I2C_BUFF_MAX_LEN 32

typedef struct i2cData {
  float maxX;
  float maxY;
  float maxZ;
  float maxMag;
  float maxMagX;
  float maxMagY;
  float maxMagZ;
}i2cData;

#define i2cDataSize 28

void reqISR(void);

void alarmMatch(void);

#endif  // _GTRACKERI2C_H
