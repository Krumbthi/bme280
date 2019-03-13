#include "i2c.h"
#include <iostream>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>

#include "macros.h"


I2cDevice::I2cDevice()
{
    Device = -1;   
    Log = Logger::GetInstance();
}

bool I2cDevice::Setup(const char* filename, int addr)
{
    unsigned long funcs;

    Log->Msg((char*)"I2cDevice::Setup");

    if ((Device = ::open(filename, O_RDWR)) < 0) {
        /* ERROR HANDLING: you can check errno to see what went wrong */
        std::cout << "Failed to open the i2c bus" << std::endl;
        //exit(1);
        return false;
    }

    if (ioctl(Device, I2C_SLAVE, addr) < 0) {
        std::cout << "Failed to acquire bus access and/or talk to slave." << std::endl;
        /* ERROR HANDLING; you can check errno to see what went wrong */
        //exit(1);
        return false;
    }

    /* Ergebnis untersuchen */
    if (funcs & I2C_FUNC_I2C)
        Log->Msg((char*)"I2cDevice::I2C");
    if (funcs & (I2C_FUNC_SMBUS_BYTE))
        Log->Msg((char*)"I2cDevice::I2C_FUNC_SMBUS_BYTE");

    return true;
}

void I2cDevice::Close()
{
    if (Device)
        ::close(Device);
}

bool I2cDevice::ReadByte(int reg, int32_t *data)
{
#ifdef LOGGING
    Log->Msg("I2cDevice::ReadByte");
#endif

#ifdef SMBUS
    *data = i2c_smbus_read_byte_data(Device, reg);
#else
    ::write(Device, &reg, 1);
    ::read(Device, data, 1);
#endif
    if (*data < 0) {
        return false;
    }
    return true;
}

bool I2cDevice::WriteByte(int reg, uint8_t *cmd)
{
    int retval;

#ifdef LOGGING    
    Log->Msg((char*)"I2cDevice::WriteByte");
#endif

#ifdef SMBUS
    retval = i2c_smbus_write_byte_data(Device, reg, *cmd);
#else
    ::write(Device, &reg, 1);
    retval = ::write(Device, cmd, 1);
#endif
    if (retval < 0)
        return false;
    return true;
}

bool I2cDevice::WriteData(int reg, uint8_t *buf, size_t bufSize)
{
    int retval = -1;

#ifdef LOGGING
    Log->Msg((char*)"I2cDevice::WriteData");
#endif

#ifdef SMBUS    
    if(bufSize == 1) {
        retval = i2c_smbus_write_byte_data(Device, reg, *buf);
    } else if (bufSize == 2) {
        retval = i2c_smbus_write_word_data(Device, reg, MAKEWORD(buf[0], buf[1]));
    } else {
        retval = i2c_smbus_write_block_data(Device, reg, bufSize, buf);
    }

    if (retval > 0) {
        hex_dump(buf, bufSize, 8, "i2cwrite");
        return true;
    }
#else
    ::write(Device, &reg, 1);
    if(::write(Device, buf, bufSize) != bufSize)
        return false;
#endif
    hex_dump(buf, bufSize, 8, "i2cwrite");
    return true;
}

uint32_t I2cDevice::ReadData(int reg)
{
    uint32_t value;

#ifdef LOGGING
    Log->Msg((char*)"I2cDevice::ReadData2");
#endif

#ifdef SMBUS    
    value = i2c_smbus_read_byte_data(Device, reg);
    value <<= 8;
    value |= i2c_smbus_read_byte_data(Device, reg);
    value <<= 8;
    value |= i2c_smbus_read_byte_data(Device, reg);
#else
    uint8_t buf[3];
    ::write(Device, &reg, 1);
    ::read(Device, buf, ARRAY_SIZE(buf));
    
    value = buf[0] << 8;
    value |= buf[1] << 8;
    value |= buf[2];
#endif    
    return value;
}

bool I2cDevice::ReadData(int reg, uint8_t *buf, size_t bufSize)
{
    int retval;

#ifdef LOGGING
    Log->Msg((char*)"I2cDevice::ReadData Native");
#endif

#ifdef SMBUS
    for (int i=0; i<=bufSize; i++) {
        buf[i] = i2c_smbus_read_byte_data(Device, reg);
    }

    switch(bufSize) {
        case 1:
            value = buf[0];
            break;
        case 2:
            value <<= 8;
            value |= buf[1];
            break;
        case 3:
            value <<= 8;
            value |= buf[2];
            break;
    }
#else
    retval = ::write(Device, &reg, 1);
    retval = ::read(Device, buf, bufSize);
#endif
    if (retval < 0)
        return false;

    hex_dump(buf, bufSize, 8, "ReadData");

    return true;
}

bool I2cDevice::ReadBuffer(int reg, uint8_t *buf, size_t bufSize)
{

#ifdef LOGGING
    Log->Msg((char*)"I2cDevice::ReadBuffer");
#endif

#ifdef SMBUS
    for (int i=0; i<=bufSize; i++) {
        buf[i] = i2c_smbus_read_byte_data(Device, reg);
        if (buf[i] < 0)
            return false;
    }
#else
    ::write(Device, &reg, 1);
    ::read(Device, buf, bufSize);
#endif
    hex_dump(buf, bufSize, 8, "i2cRead");
    return true;
}

