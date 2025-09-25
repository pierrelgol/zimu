#ifndef I2C_H
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

extern int I2CRead(int fd);
extern int I2CReadReg8(int fd, int reg);
extern int I2CReadReg16(int fd, int reg);
extern int I2CReadBlock(int fd, int reg, unsigned char *data, int datalen);

extern int I2CWrite(int fd, int data);
extern int I2CWriteReg8(int fd, int reg, int data);
extern int I2CWriteReg16(int fd, int reg, int data);
extern int I2CWriteBlock(int fd, int reg, const unsigned char *data, int datalen);

extern int I2CSetupInterface(const char *device, int devId);
extern int I2CSetup(const int devId);

#ifdef __cplusplus
}
#endif
#endif
