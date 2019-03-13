#ifndef __I2CDEVICE_h
#define __I2CDEVICE_h

#include <stdint.h>
#include <stdlib.h>
#include "logger.h"
#include <memory>

class I2cDevice
{
    public:
        I2cDevice();
        
        bool Setup(const char* filename, int addr);
        void Close();

        //bool ReadData(int reg, uint8_t *buf, size_t bufSize);
        uint32_t ReadData(int reg);
        bool ReadData(int reg, uint8_t *buf, size_t bufSize);
        bool ReadByte(int reg, int32_t *data);
        bool ReadBuffer(int reg, uint8_t *buf, size_t bufSize);

        bool WriteData(int reg, uint8_t *buf, size_t bufSize);
        bool WriteByte(int reg, uint8_t *cmd);

    private:
        int Device;
        Logger *Log;
};

#endif
