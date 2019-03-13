#ifndef __BME280_h
#define __BME280_h

#include "bme280_defs.h"
#include "i2c.h"
#include "logger.h"

class BME280
{
    public:
        BME280();
        ~BME280();

        bool   Init(const char* dev);
        double GetTemperature();
        double GetHumidity();
        double GetPressure();
        double GetDewPoint();
        void   Process();
   
        /*!
        * @brief This API sets the oversampling, filter and standby duration
        * (normal mode) settings in the sensor.
        *
        * @param[in] dev : Structure instance of bme280_dev.
        * @param[in] desired_settings : Variable used to select the settings which
        * are to be set in the sensor.
        *
        * @note : Below are the macros to be used by the user for selecting the
        * desired settings. User can do OR operation of these macros for configuring
        * multiple settings.
        *
        * Macros		  |   Functionality
        * ------------------------|----------------------------------------------
        * BME280_OSR_PRESS_SEL    |   To set pressure oversampling.
        * BME280_OSR_TEMP_SEL     |   To set temperature oversampling.
        * BME280_OSR_HUM_SEL      |   To set humidity oversampling.
        * BME280_FILTER_SEL       |   To set filter setting.
        * BME280_STANDBY_SEL      |   To set standby duration setting.
        *
        * @return Result of API execution status
        * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
        */
        int8_t SetSensorSettings(uint8_t desired_settings);

        /*!
        * @brief This API gets the oversampling, filter and standby duration
        * (normal mode) settings from the sensor.
        *
        * @param[in,out] dev : Structure instance of bme280_dev.
        *
        * @return Result of API execution status
        * @retval zero -> Success / +ve value -> Warning / -ve value -> Error.
        */
        int8_t GetSensorSettings();

        /*!
        * @brief This API sets the power mode of the sensor.
        *
        * @param[in] dev : Structure instance of bme280_dev.
        * @param[in] sensor_mode : Variable which contains the power mode to be set.
        *
        *    sensor_mode       |   Macros
        * ---------------------|-------------------
        *     0                | BME280_SLEEP_MODE
        *     1                | FORCED_MODE
        *     3                | BME280_NORMAL_MODE
        *
        * @return Result of API execution status
        * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
        */
        int8_t SetSensorMode(uint8_t sensor_mode);

        /*!
        * @brief This API gets the power mode of the sensor.
        *
        * @param[in] dev : Structure instance of bme280_dev.
        * @param[out] sensor_mode : Pointer variable to store the power mode.
        *
        *   sensor_mode        |   Macros
        * ---------------------|-------------------
        *     0                | BME280_SLEEP_MODE
        *     1                | BME280_FORCED_MODE
        *     3                | BME280_NORMAL_MODE
        *
        * @return Result of API execution status
        * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
        */
        bool    GetSensorMode(uint8_t *sensor_mode);
        
        /*!
        * @brief This API reads the pressure, temperature and humidity data from the
        * sensor, compensates the data and store it in the bme280_data structure
        * instance passed by the user.
        *
        * @param[in] sensor_comp : Variable which selects which data to be read from
        * the sensor.
        *
        * sensor_comp |   Macros
        * ------------|-------------------
        *     1       | BME280_PRESS
        *     2       | BME280_TEMP
        *     4       | BME280_HUM
        *     7       | BME280_ALL
        *
        * @param[out] comp_data : Structure instance of bme280_data.
        * @param[in] dev : Structure instance of bme280_dev.
        *
        * @return Result of API execution status
        * @retval zero -> Success / +ve value -> Warning / -ve value -> Error
        */
        int8_t  GetSensorData(uint8_t sensor_comp);

    private:
        bool    SoftReset();

        /*!
        *  @brief This API is used to parse the pressure, temperature and
        *  humidity data and store it in the bme280_uncomp_data structure instance.
        *
        *  @param[in] reg_data     : Contains register data which needs to be parsed
        *  @param[out] uncomp_data : Contains the uncompensated pressure, temperature
        *  and humidity data.
        */
        void    ParseSensorData(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data);

        /*!
        * @brief This API is used to compensate the pressure and/or
        * temperature and/or humidity data according to the component selected by the
        * user.
        *
        * @param[in] sensor_comp : Used to select pressure and/or temperature and/or
        * humidity.
        * @param[in] uncomp_data : Contains the uncompensated pressure, temperature and
        * humidity data.
        * @param[out] comp_data : Contains the compensated pressure and/or temperature
        * and/or humidity data.
        * @param[in] calib_data : Pointer to the calibration data structure.
        *
        * @return Result of API execution status.
        * @retval zero -> Success / -ve value -> Error
        */
        int8_t  CompensateData(uint8_t sensor_comp, const TUncompData *uncomp_data);

        int8_t  SetOsrSettings(uint8_t desired_settings);
        int8_t  SetOsrHumiditySettings();
        int8_t  SetOsrPressTempSettings(uint8_t desired_settings);
        int8_t  SetFilterStandbySettings(uint8_t desired_settings);
        void    FillFilterSettings(uint8_t *reg_data);
        void    FillStandbySettings(uint8_t *reg_data);
        void    FillOsrPressSettings(uint8_t *reg_data);
        void    FillOsrTempSettings(uint8_t *reg_data);
        void    ParseDeviceSettings(const uint8_t *reg_data);
        int8_t  WritePowerMode(uint8_t sensor_mode);
        int8_t  PutDevice2Sleep();
        int8_t  ReloadDeviceSettings();

#ifdef BME280_FLOAT_ENABLE
        double  CompensateHumidity(const struct bme280_uncomp_data *uncomp_data);
        double  CompensatePressure(const struct bme280_uncomp_data *uncomp_data);
        double  CompensateTemperature(const struct bme280_uncomp_data *uncomp_data);
#else
        uint32_t CompensateHumidity(const struct bme280_uncomp_data *uncomp_data);
        uint32_t CompensatePressure(const struct bme280_uncomp_data *uncomp_data);
        int32_t  CompensateTemperature(const struct bme280_uncomp_data *uncomp_data);
#endif
        bool    GetCalibData();
        bool    ReadTrim();

        uint8_t AreSettingsChanged(uint8_t sub_settings, uint8_t desired_settings);
        void    ParseHumidityCalibData(const uint8_t *reg_data);
        void    ParseTempCalibData(const uint8_t *reg_data);
        void    InterleaveRegAddr(const uint8_t *reg_addr, uint8_t *temp_buff, const uint8_t *reg_data, uint8_t len);
        
        const char  *IfName; 
        uint8_t     ChipId;
        uint8_t     DevId;
        
        std::shared_ptr<I2cDevice>  IFace;
        std::shared_ptr<TCalibData> CalibData;
        std::shared_ptr<TSettings>  Settings;
        std::shared_ptr<TData>      Data;
        Logger                      *Log;

        //std::shared_ptr<Logger>     Log;
};

#endif
