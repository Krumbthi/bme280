#include "bme280.h"
#include <unistd.h>
#include "macros.h"
#include <math.h>


#define OVERSAMPLING_SETTINGS		UINT8_C(0x07)
// To identify filter and standby settings selected by user
#define FILTER_STANDBY_SETTINGS	    UINT8_C(0x18)
#define SENSOR_SLEEP_TIME_uS		300000


BME280::BME280()
{
    Data = std::make_shared<TData>();
	Settings = std::make_shared<TSettings>();

	CalibData = std::make_shared<TCalibData>();
    IFace = std::make_shared<I2cDevice>(); 
	
	Log = Logger::GetInstance();
	
	Data->temperature = 0.0;
	Data->pressure = 0.0;
	Data->humidity = 0.0;
}

BME280::~BME280()
{
	IFace->Close();
}

/*!
 *  @brief This API is the entry point.
 *  It reads the chip-id and calibration data from the sensor.
 */
bool BME280::Init(const char *dev)
{
	bool rslt;
	
    // chip id read try count
	uint8_t try_count = 5;
	int32_t chip_id = 0;

#ifdef LOGGING	
    Log->Msg((char*)"BME280::Init");

	char msg[40];
#endif
    rslt = IFace->Setup(dev, BME280_I2C_ADDR_SEC);

    uint8_t osrs_t = BME280_OVERSAMPLING_1X;            // Temperature oversampling x 1
    uint8_t osrs_p = BME280_OVERSAMPLING_1X;            // Pressure oversampling x 1
    uint8_t osrs_h = BME280_OVERSAMPLING_1X;            // Humidity oversampling x 1
    uint8_t mode   = BME280_FORCED_MODE;                // Normal mode
    uint8_t t_sb   = BME280_STANDBY_TIME_125_MS;        // Tstandby 1000ms
    uint8_t filter = BME280_FILTER_COEFF_OFF;           // Filter off
    uint8_t spi3w_en = 0;                               // 3-wire SPI Disable

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
    uint8_t ctrl_hum_reg  = osrs_h;

    // Proceed if null check is fine
	if (rslt) {
		while (try_count) {
			// Read the chip-id of bme280 sensor 
			if (IFace->ReadByte(BME280_CHIP_ID_ADDR, &chip_id)) {
				// Check for chip id validity 
				if (chip_id == BME280_CHIP_ID) {
					ChipId = chip_id;
#ifdef LOGGING
					sprintf(msg, "BME280::ChipId 0x%02X", ChipId);
					Log->Msg(msg);
#endif
					IFace->WriteData(BME280_CTRL_HUM_ADDR, &ctrl_hum_reg, 1);
					IFace->WriteData(BME280_CTRL_MEAS_ADDR, &ctrl_meas_reg, 1);
					IFace->WriteData(BME280_CONFIG_ADDR, &config_reg, 1);

                    if (rslt) {
                        //rslt = this->ReadTrim();
                        rslt = this->GetCalibData();
                    }
                    //this->SetSensorSettings(settings);

					break;
				}
			}
			// Wait for 1 ms 
            usleep(1000);

			--try_count;
		}

		// Chip id check failed 
		if (!try_count)
			rslt = BME280_E_DEV_NOT_FOUND;
	}

	return rslt;
}

double BME280::GetTemperature()
{
#ifdef ALT_MEAS
	int32_t var1, var2;
	uint8_t reg_data[3];
	TUncompData uncomp_data = {0, };

    //int32_t adc_T = IFace->ReadData(BME280_REGISTER_TEMPDATA);
    IFace->ReadData(BME280_REGISTER_TEMPDATA, reg_data, ARRAY_SIZE(reg_data));
    /* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	// Store the parsed register values for pressure data 
	data_msb = (uint32_t)reg_data[0] << 12;
	data_lsb = (uint32_t)reg_data[1] << 4;
	data_xlsb = (uint32_t)reg_data[2] >> 4;
	
	int32_t adc_T = data_msb | data_lsb | data_xlsb;

    if (adc_T == 0x800000) { // value in case temp measurement was disabled
#ifdef LOGGING
        Log->Msg((char *) "Temp disabled");
#endif
        return 0.0;
    }

	adc_T >>= 4;

    var1 = ((((adc_T>>3) - ((int32_t)CalibData->dig_T1 <<1))) * ((int32_t)CalibData->dig_T2)) >> 11;        
    var2 = (((((adc_T>>4) - ((int32_t)CalibData->dig_T1)) * 
              ((adc_T>>4) - ((int32_t)CalibData->dig_T1))) >> 12) * 
			  ((int32_t)CalibData->dig_T3)) >> 14;

    CalibData->t_fine = var1 + var2;

    double t = (CalibData->t_fine * 5 + 128) >> 8;
	Data->temperature = t/100;
#endif

#ifdef LOGGING
	char msg[40];
    sprintf(msg, "BME280::Temp %f", Data->temperature);
	Log->Msg(msg);
#endif
	
	return Data->temperature;
}

double BME280::GetHumidity()
{
#ifdef ALT_MEAS
	this->GetTemperature(); // must be done first to get t_fine
	uint8_t buf[2];

    int32_t adc_H = IFace->ReadData(BME280_REGISTER_HUMIDDATA, buf, ARRAY_SIZE(buf));

    if (adc_H == 0x8000) { // value in case humidity measurement was disabled
#ifdef LOGGING
        Log->Msg((char *) "Hum disabled");
#endif
        return 0.0;
    }
        
    int32_t v_x1_u32r;

    v_x1_u32r = (CalibData->t_fine - ((int32_t)76800));

    v_x1_u32r = (((((adc_H << 14) - (((int32_t)CalibData->dig_H4) << 20) -
                    (((int32_t)CalibData->dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) *
                 (((((((v_x1_u32r * ((int32_t)CalibData->dig_H6)) >> 10) *
                      (((v_x1_u32r * ((int32_t)CalibData->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                    ((int32_t)2097152)) * ((int32_t)CalibData->dig_H2) + 8192) >> 14));

    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)CalibData->dig_H1)) >> 4));

    v_x1_u32r = (v_x1_u32r < 0) ? 0 : v_x1_u32r;
    v_x1_u32r = (v_x1_u32r > 419430400) ? 419430400 : v_x1_u32r;
    double h = (v_x1_u32r>>12);
	
	Data->humidity = h / 1024.0;
#endif    
#ifdef LOGGING
	char msg[40];
    sprintf(msg, "BME280::Hum %f", Data->humidity);
	Log->Msg(msg);
#endif

	return Data->humidity;
}

double BME280::GetPressure()
{
#ifdef ALT_MEAS
	int64_t var1, var2, p;
	uint8_t buf[3];

    this->GetTemperature(); // must be done first to get t_fine

    int32_t adc_P = IFace->ReadData(BME280_REGISTER_PRESSUREDATA, buf, ARRAY_SIZE(buf));
    if (adc_P == 0x800000) { // value in case pressure measurement was disabled
        return 0.0;
#ifdef LOGGING
        Log->Msg((char *) "Pres disabled");
#endif
    }
    
	adc_P >>= 4;

    var1 = ((int64_t)CalibData->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)CalibData->dig_P6;
    var2 = var2 + ((var1*(int64_t)CalibData->dig_P5)<<17);
    var2 = var2 + (((int64_t)CalibData->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)CalibData->dig_P3)>>8) + ((var1 * (int64_t)CalibData->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)CalibData->dig_P1)>>33;

    if (var1 == 0) {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - adc_P;
    p = (((p<<31) - var2) * 3125) / var1;
    var1 = (((int64_t)CalibData->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)CalibData->dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)CalibData->dig_P7)<<4);
	Data->pressure = (double) p / 256;
#endif	
#ifdef LOGGING
	char msg[40];
    sprintf(msg, "BME280::Pres %f", Data->pressure);
	Log->Msg(msg);
#endif
    return Data->pressure;
}

//****************************************************************************//
// Returns Dew point in DegC
double BME280::GetDewPoint()
{
	double celsius = this->GetTemperature(); 
  	double humidity = this->GetHumidity();

  	// (1) Saturation Vapor Pressure = ESGG(T)
  	double RATIO = 373.15 / (273.15 + celsius);
  	double RHS = -7.90298 * (RATIO - 1);
  	RHS += 5.02808 * log10(RATIO);
  	RHS += -1.3816e-7 * (pow(10, (11.344 * (1 - 1/RATIO ))) - 1) ;
  	RHS += 8.1328e-3 * (pow(10, (-3.49149 * (RATIO - 1))) - 1) ;
  	RHS += log10(1013.246);
    // factor -3 is to adjust units - Vapor Pressure SVP * humidity
  	double VP = pow(10, RHS - 3) * humidity;
	// (2) DEWPOINT = F(Vapor Pressure)
  	double T = log(VP/0.61078);   // temp var
  
  	return (241.88 * T) / (17.558 - T);
}


void BME280::Process()
{
	uint8_t comp_data;
	this->SetSensorMode(BME280_NORMAL_MODE);
    // Wait for the measurement to complete and print data @25Hz 
	usleep(1000*40);
	this->GetSensorData(comp_data);
}

int8_t BME280::SetSensorSettings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t sensor_mode;

    rslt = GetSensorMode(&sensor_mode);

    if (rslt) {
		if (sensor_mode != BME280_SLEEP_MODE)
			rslt = PutDevice2Sleep();

		if (rslt) {
			// Check if user wants to change oversampling settings 			
			if (AreSettingsChanged(OVERSAMPLING_SETTINGS, desired_settings))
				rslt = SetOsrSettings(desired_settings);

			// Check if user wants to change filter and/or standby settings 
			if ((rslt == BME280_OK) && AreSettingsChanged(FILTER_STANDBY_SETTINGS, desired_settings))
				rslt = SetFilterStandbySettings(desired_settings);
		}
	}

	return rslt;
}

int8_t BME280::GetSensorSettings()
{
	int8_t rslt;
	uint8_t reg_data[4];

    // Proceed if null check is fine
    rslt = IFace->ReadData(BME280_CTRL_HUM_ADDR, reg_data, ARRAY_SIZE(reg_data));
    if (rslt) {
		ParseDeviceSettings(reg_data);
	}

	return rslt;
}

int8_t BME280::SetSensorMode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t last_set_mode;
	
    rslt = GetSensorMode(&last_set_mode);
    if(rslt) {
		// If the sensor is not in sleep mode put the device to sleep mode		
		if (last_set_mode != BME280_SLEEP_MODE)
			rslt = PutDevice2Sleep();

		// Set the power mode 
		if (rslt)
			rslt = WritePowerMode(sensor_mode);
	}

	return rslt;
}

/*!
 * @brief This API gets the power mode of the sensor.
 */
bool BME280::GetSensorMode(uint8_t *sensor_mode)
{
	bool rslt;
    rslt = IFace->ReadData(BME280_PWR_CTRL_ADDR, sensor_mode, 1);

    if (rslt) {
		// Assign the power mode in the device structure 
		*sensor_mode = BME280_GET_BITS_POS_0(*sensor_mode, BME280_SENSOR_MODE);
	}

	return rslt;
}

/*!
 * @brief This API performs the soft reset of the sensor.
 */
bool BME280::SoftReset()
{
	bool rslt;
	uint8_t reg_addr = BME280_RESET_ADDR;

	// 0xB6 is the soft reset command 
	uint8_t soft_rst_cmd = 0xB6;

#ifdef LOGGING
	Log->Msg((char*)"BME280::SoftReset");
#endif

	// Write the soft reset command in the sensor */
	rslt = IFace->WriteByte(reg_addr, &soft_rst_cmd);
#ifdef LOGGING
	if(rslt) {
		Log->Msg((char*)"BME280::SoftReset->done");
	}
#endif	
	usleep(SENSOR_SLEEP_TIME_uS);

	return rslt;
}

/*!
 * @brief This API reads the pressure, temperature and humidity data from the
 * sensor, compensates the data and store it in the bme280_data structure
 * instance passed by the user.
 */
int8_t BME280::GetSensorData(uint8_t sensor_comp)
{
	int8_t rslt;
	// Array to store the pressure, temperature and humidity data read from	the sensor 
	uint8_t reg_data[BME280_P_T_H_DATA_LEN] = {0};
    TUncompData uncomp_data = {0, };

	if (Data != NULL) {
        // Read the pressure and temperature data from the sensor
        rslt = IFace->ReadData(BME280_DATA_ADDR, reg_data, BME280_P_T_H_DATA_LEN);
        if (rslt) {
			// Parse the read data from the sensor 
			ParseSensorData(reg_data, &uncomp_data);
			// Compensate the pressure and/or temperature and/or humidity data from the sensor */
			rslt = CompensateData(sensor_comp, &uncomp_data);
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 *  @brief This API is used to parse the pressure, temperature and
 *  humidity data and store it in the bme280_uncomp_data structure instance.
 */
void BME280::ParseSensorData(const uint8_t *reg_data, struct bme280_uncomp_data *uncomp_data)
{
	/* Variables to store the sensor data */
	uint32_t data_xlsb;
	uint32_t data_lsb;
	uint32_t data_msb;

	// Store the parsed register values for pressure data 
	data_msb = (uint32_t)reg_data[0] << 12;
	data_lsb = (uint32_t)reg_data[1] << 4;
	data_xlsb = (uint32_t)reg_data[2] >> 4;
	uncomp_data->pressure = data_msb | data_lsb | data_xlsb;
	
	// Store the parsed register values for temperature data 
	data_msb = (uint32_t)reg_data[3] << 12;
	data_lsb = (uint32_t)reg_data[4] << 4;
	data_xlsb = (uint32_t)reg_data[5] >> 4;
	uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
	
	// Store the parsed register values for temperature data 
	data_lsb = (uint32_t)reg_data[6] << 8;
	data_msb = (uint32_t)reg_data[7];
	uncomp_data->humidity = data_msb | data_lsb;

#ifdef LOGGING
    char msg[40];
    sprintf(msg, "BME280::Pres %f", uncomp_data->pressure);
    Log->Msg(msg);
    sprintf(msg, "BME280::Temp %f", uncomp_data->temperature);
    Log->Msg(msg);
    sprintf(msg, "BME280::Hum %f", uncomp_data->humidity);
    Log->Msg(msg);
#endif
}

/*!
 * @brief This API is used to compensate the pressure and/or
 * temperature and/or humidity data according to the component selected
 * by the user.
 */
int8_t BME280::CompensateData(uint8_t sensor_comp, const TUncompData *uncomp_data)
{
	int8_t rslt = BME280_OK;

	if ((uncomp_data != NULL) && (Data != NULL) && (CalibData != NULL)) {
		// Initialize to zero 
		Data->temperature = 0;
		Data->pressure = 0;
		Data->humidity = 0;

		// If pressure or temperature component is selected 
		if (sensor_comp & (BME280_PRESS | BME280_TEMP | BME280_HUM)) {
			// Compensate the temperature data 
			Data->temperature = CompensateTemperature(uncomp_data);
		}
		if (sensor_comp & BME280_PRESS) {
			// Compensate the pressure data 
			Data->pressure = CompensatePressure(uncomp_data);
		}
		if (sensor_comp & BME280_HUM) {
			// Compensate the humidity data 
			Data->humidity = CompensateHumidity(uncomp_data);
		}
	} else {
		rslt = BME280_E_NULL_PTR;
	}

	return rslt;
}

/*!
 * @brief This internal API sets the oversampling settings for pressure,
 * temperature and humidity in the sensor.
 */
int8_t BME280::SetOsrSettings(uint8_t desired_settings)
{
	int8_t rslt = BME280_W_INVALID_OSR_MACRO;

#ifdef LOGGING
    Log->Msg((char*)"BME280::SetOsrSettings");
#endif

	if (desired_settings & BME280_OSR_HUM_SEL)
		rslt = SetOsrHumiditySettings();
	if (desired_settings & (BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL))
		rslt = SetOsrPressTempSettings(desired_settings);

	return rslt;
}

/*!
 * @brief This API sets the humidity oversampling settings of the sensor.
 */
int8_t BME280::SetOsrHumiditySettings()
{
	int8_t rslt;
	uint8_t ctrl_hum;
	uint8_t ctrl_meas;
	uint8_t reg_addr = BME280_CTRL_HUM_ADDR;

#ifdef LOGGING
    Log->Msg((char*)"BME280::SetOsrHumiditySettings");
#endif

    ctrl_hum = Settings->osr_h & BME280_CTRL_HUM_MSK;
	// Write the humidity control value in the register 
	//rslt = bme280_set_regs(&reg_addr, &ctrl_hum, 1, dev);
	rslt = IFace->WriteData((int)reg_addr, &ctrl_hum, 1);
	// Humidity related changes will be only effective after a write operation to ctrl_meas register 
	if (rslt == BME280_OK) {
		reg_addr = BME280_CTRL_MEAS_ADDR;
		//rslt = bme280_get_regs(reg_addr, &ctrl_meas, 1, dev);
		rslt = IFace->ReadData(reg_addr, &ctrl_meas, 1);

		if (rslt == BME280_OK) {
			//rslt = bme280_set_regs(&reg_addr, &ctrl_meas, 1, dev);
			rslt = IFace->WriteData((int)reg_addr, &ctrl_meas, 1);
		}
	}

	return rslt;
}

/*!
 * @brief This API sets the pressure and/or temperature oversampling settings
 * in the sensor according to the settings selected by the user.
 */
int8_t BME280::SetOsrPressTempSettings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_CTRL_MEAS_ADDR;
	uint8_t reg_data;

#ifdef LOGGING
    Log->Msg((char*)"BME280::SetOsrPressTempSettings");
#endif
    rslt = IFace->ReadData(reg_addr, &reg_data, 1);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_OSR_PRESS_SEL)
			FillOsrPressSettings(&reg_data);
		if (desired_settings & BME280_OSR_TEMP_SEL)
			FillOsrTempSettings(&reg_data);
		// Write the oversampling settings in the register 
		//rslt = bme280_set_regs(&reg_addr, &reg_data, 1, dev);
		rslt = IFace->WriteData(reg_addr, &reg_data, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API sets the filter and/or standby duration settings
 * in the sensor according to the settings selected by the user.
 */
int8_t BME280::SetFilterStandbySettings(uint8_t desired_settings)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_CONFIG_ADDR;
	uint8_t reg_data;

#ifdef LOGGING
    Log->Msg((char*)"BME280::SetFilterStandbySettings");
#endif
	//rslt = GetRegs(reg_addr, &reg_data, 1, dev);
    rslt = IFace->ReadData(reg_addr, &reg_data, 1);

	if (rslt == BME280_OK) {
		if (desired_settings & BME280_FILTER_SEL)
			FillFilterSettings(&reg_data);
		if (desired_settings & BME280_STANDBY_SEL)
			FillStandbySettings(&reg_data);
		// Write the oversampling settings in the register 
		//rslt = SetRegs(&reg_addr, &reg_data, 1, dev);
		rslt = IFace->WriteData(reg_addr, &reg_data, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API fills the filter settings provided by the user
 * in the data buffer so as to write in the sensor.
 */
void BME280::FillFilterSettings(uint8_t *reg_data)
{
#ifdef LOGGING
    Log->Msg((char*)"BME280::FillFilterSettings");
#endif
	*reg_data = BME280_SET_BITS(*reg_data, BME280_FILTER, Settings->filter);
}

/*!
 * @brief This internal API fills the standby duration settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void BME280::FillStandbySettings(uint8_t *reg_data)
{
#ifdef LOGGING
    Log->Msg((char*)"BME280::FillStandbySettings");
#endif
	*reg_data = BME280_SET_BITS(*reg_data, BME280_STANDBY, Settings->standby_time);
}

/*!
 * @brief This internal API fills the pressure oversampling settings provided by
 * the user in the data buffer so as to write in the sensor.
 */
void BME280::FillOsrPressSettings(uint8_t *reg_data)
{
#ifdef LOGGING
    Log->Msg((char*)"BME280::FillOsrPressSettings");
#endif
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_PRESS, Settings->osr_p);
}

/*!
 * @brief This internal API fills the temperature oversampling settings
 * provided by the user in the data buffer so as to write in the sensor.
 */
void BME280::FillOsrTempSettings(uint8_t *reg_data)
{
#ifdef LOGGING
    Log->Msg((char*)"BME280::FillOsrTempSettings");
#endif
	*reg_data = BME280_SET_BITS(*reg_data, BME280_CTRL_TEMP, Settings->osr_t);
}

/*!
 * @brief This internal API parse the oversampling(pressure, temperature
 * and humidity), filter and standby duration settings and store in the
 * device structure.
 */
void BME280::ParseDeviceSettings(const uint8_t *reg_data)
{
#ifdef LOGGING
    Log->Msg((char*)"BME280::ParseDeviceSettings");
#endif
	Settings->osr_h = BME280_GET_BITS_POS_0(reg_data[0], BME280_CTRL_HUM);
	Settings->osr_p = BME280_GET_BITS(reg_data[2], BME280_CTRL_PRESS);
	Settings->osr_t = BME280_GET_BITS(reg_data[2], BME280_CTRL_TEMP);
	Settings->filter = BME280_GET_BITS(reg_data[3], BME280_FILTER);
	Settings->standby_time = BME280_GET_BITS(reg_data[3], BME280_STANDBY);
}
/*!
 * @brief This internal API writes the power mode in the sensor.
 */
int8_t BME280::WritePowerMode(uint8_t sensor_mode)
{
	int8_t rslt;
	uint8_t reg_addr = BME280_PWR_CTRL_ADDR;
	/* Variable to store the value read from power mode register */
	uint8_t sensor_mode_reg_val;

#ifdef LOGGING
    Log->Msg((char*)"BME280::WritePowerMode");
#endif

	// Read the power mode register 
	//rslt = bme280_get_regs(reg_addr, &sensor_mode_reg_val, 1, dev);
	rslt = IFace->ReadData(reg_addr, &sensor_mode_reg_val, 1);
	// Set the power mode 

	if (rslt == BME280_OK) {
		sensor_mode_reg_val = BME280_SET_BITS_POS_0(sensor_mode_reg_val, BME280_SENSOR_MODE, sensor_mode);
		// Write the power mode in the register 
		rslt = IFace->WriteData(reg_addr, &sensor_mode_reg_val, 1);
	}

	return rslt;
}

/*!
 * @brief This internal API puts the device to sleep mode.
 */
int8_t BME280::PutDevice2Sleep()
{
	int8_t rslt;
	uint8_t reg_data[4];

#ifdef LOGGING
    Log->Msg((char*)"BME280::PutDevice2Sleep");
#endif

	//rslt = GetRegs(BME280_CTRL_HUM_ADDR, reg_data, 4, dev);
	rslt = IFace->ReadData(BME280_CTRL_HUM_ADDR, reg_data, ARRAY_SIZE(reg_data));
	if (rslt) {
		ParseDeviceSettings(reg_data);
		rslt = SoftReset();
        if (rslt) {
			rslt = ReloadDeviceSettings();
			return BME280_OK;
        }
	}

	return BME280_E_SLEEP_MODE_FAIL;
}

/*!
 * @brief This internal API reloads the already existing device settings in
 * the sensor after soft reset.
 */
int8_t BME280::ReloadDeviceSettings()
{
	int8_t rslt;

	rslt = SetOsrSettings(BME280_ALL_SETTINGS_SEL);
	if (rslt)
		rslt = SetFilterStandbySettings(BME280_ALL_SETTINGS_SEL);

	return rslt;
}

#ifdef BME280_FLOAT_ENABLE
/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in double data type.
 */
double BME280::CompensateTemperature(const struct bme280_uncomp_data *uncomp_data)
{
	double var1;
	double var2;
	double temperature;
	double temperature_min = -40;
	double temperature_max = 85;

	var1 = ((double)uncomp_data->temperature) / 16384.0 - ((double)CalibData->dig_T1) / 1024.0;
	var1 = var1 * ((double)CalibData->dig_T2);
	var2 = (((double)uncomp_data->temperature) / 131072.0 - ((double)CalibData->dig_T1) / 8192.0);
	var2 = (var2 * var2) * ((double)CalibData->dig_T3);
	CalibData->t_fine = (int32_t)(var1 + var2);
	temperature = (var1 + var2) / 5120.0;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}

/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in double data type.
 */
double BME280::CompensatePressure(const struct bme280_uncomp_data *uncomp_data)
{
	double var1;
	double var2;
	double var3;
	double pressure;
	double pressure_min = 30000.0;
	double pressure_max = 110000.0;

	var1 = ((double)CalibData->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)CalibData->dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)CalibData->dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)CalibData->dig_P4) * 65536.0);
	var3 = ((double)CalibData->dig_P3) * var1 * var1 / 524288.0;
	var1 = (var3 + ((double)CalibData->dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)CalibData->dig_P1);
	// avoid exception caused by division by zero 
	if (var1) {
		pressure = 1048576.0 - (double) uncomp_data->pressure;
		pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
		var1 = ((double)CalibData->dig_P9) * pressure * pressure / 2147483648.0;
		var2 = pressure * ((double)CalibData->dig_P8) / 32768.0;
		pressure = pressure + (var1 + var2 + ((double)CalibData->dig_P7)) / 16.0;

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else { /* Invalid case */
		pressure = pressure_min;
	}

	return pressure;
}

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in double data type.
 */
double BME280::CompensateHumidity(const struct bme280_uncomp_data *uncomp_data)
{
	double humidity;
	double humidity_min = 0.0;
	double humidity_max = 100.0;
	double var1;
	double var2;
	double var3;
	double var4;
	double var5;
	double var6;

	var1 = ((double)CalibData->t_fine) - 76800.0;
	var2 = (((double)CalibData->dig_H4) * 64.0 + (((double)CalibData->dig_H5) / 16384.0) * var1);
	var3 = uncomp_data->humidity - var2;
	var4 = ((double)CalibData->dig_H2) / 65536.0;
	var5 = (1.0 + (((double)CalibData->dig_H3) / 67108864.0) * var1);
	var6 = 1.0 + (((double)CalibData->dig_H6) / 67108864.0) * var1 * var5;
	var6 = var3 * var4 * (var5 * var6);
	humidity = var6 * (1.0 - ((double)CalibData->dig_H1) * var6 / 524288.0);

	if (humidity > humidity_max)
		humidity = humidity_max;
	else if (humidity < humidity_min)
		humidity = humidity_min;

	return humidity;
}

#else
/*!
 * @brief This internal API is used to compensate the raw temperature data and
 * return the compensated temperature data in integer data type.
 */
int32_t BME280::CompensateTemperature(const struct bme280_uncomp_data *uncomp_data)
{
	int32_t var1;
	int32_t var2;
	int32_t temperature;
	int32_t temperature_min = -4000;
	int32_t temperature_max = 8500;

	var1 = (int32_t)((uncomp_data->temperature / 8) - ((int32_t)CalibData->dig_T1 * 2));
	var1 = (var1 * ((int32_t)CalibData->dig_T2)) / 2048;
	var2 = (int32_t)((uncomp_data->temperature / 16) - ((int32_t)CalibData->dig_T1));
	var2 = (((var2 * var2) / 4096) * ((int32_t)CalibData->dig_T3)) / 16384;
	CalibData->t_fine = var1 + var2;
	temperature = (CalibData->t_fine * 5 + 128) / 256;

	if (temperature < temperature_min)
		temperature = temperature_min;
	else if (temperature > temperature_max)
		temperature = temperature_max;

	return temperature;
}
#ifdef BME280_64BIT_ENABLE
/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type with higher
 * accuracy.
 */
uint32_t BME280::CompensatePressure(const struct bme280_uncomp_data *uncomp_data)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int64_t var4;
	uint32_t pressure;
	uint32_t pressure_min = 3000000;
	uint32_t pressure_max = 11000000;

	var1 = ((int64_t)CalibData->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)CalibData->dig_P6;
	var2 = var2 + ((var1 * (int64_t)CalibData->dig_P5) * 131072);
	var2 = var2 + (((int64_t)CalibData->dig_P4) * 34359738368);
	var1 = ((var1 * var1 * (int64_t)CalibData->dig_P3) / 256) + ((var1 * ((int64_t)CalibData->dig_P2) * 4096));
	var3 = ((int64_t)1) * 140737488355328;
	var1 = (var3 + var1) * ((int64_t)CalibData->dig_P1) / 8589934592;

	/* To avoid divide by zero exception */
	if (var1 != 0) {
		var4 = 1048576 - uncomp_data->pressure;
		var4 = (((var4 * 2147483648) - var2) * 3125) / var1;
		var1 = (((int64_t)CalibData->dig_P9) * (var4 / 8192) * (var4 / 8192)) / 33554432;
		var2 = (((int64_t)CalibData->dig_P8) * var4) / 524288;
		var4 = ((var4 + var1 + var2) / 256) + (((int64_t)CalibData->dig_P7) * 16);
		pressure = (uint32_t)(((var4 / 2) * 100) / 128);

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#else
/*!
 * @brief This internal API is used to compensate the raw pressure data and
 * return the compensated pressure data in integer data type.
 */
uint32_t BME280::CompensatePressure(const struct bme280_uncomp_data *uncomp_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	uint32_t var5;
	uint32_t pressure;
	uint32_t pressure_min = 30000;
	uint32_t pressure_max = 110000;

	var1 = (((int32_t)CalibData->t_fine) / 2) - (int32_t)64000;
	var2 = (((var1 / 4) * (var1 / 4)) / 2048) * ((int32_t)CalibData->dig_P6);
	var2 = var2 + ((var1 * ((int32_t)CalibData->dig_P5)) * 2);
	var2 = (var2 / 4) + (((int32_t)CalibData->dig_P4) * 65536);
	var3 = (CalibData->dig_P3 * (((var1 / 4) * (var1 / 4)) / 8192)) / 8;
	var4 = (((int32_t)CalibData->dig_P2) * var1) / 2;
	var1 = (var3 + var4) / 262144;
	var1 = (((32768 + var1)) * ((int32_t)CalibData->dig_P1)) / 32768;
	 /* avoid exception caused by division by zero */
	if (var1) {
		var5 = (uint32_t)((uint32_t)1048576) - uncomp_data->pressure;
		pressure = ((uint32_t)(var5 - (uint32_t)(var2 / 4096))) * 3125;
		if (pressure < 0x80000000)
			pressure = (pressure << 1) / ((uint32_t)var1);
		else
			pressure = (pressure / (uint32_t)var1) * 2;

		var1 = (((int32_t)CalibData->dig_P9) * ((int32_t)(((pressure / 8) * (pressure / 8)) / 8192))) / 4096;
		var2 = (((int32_t)(pressure / 4)) * ((int32_t)CalibData->dig_P8)) / 8192;
		pressure = (uint32_t)((int32_t)pressure + ((var1 + var2 + CalibData->dig_P7) / 16));

		if (pressure < pressure_min)
			pressure = pressure_min;
		else if (pressure > pressure_max)
			pressure = pressure_max;
	} else {
		pressure = pressure_min;
	}

	return pressure;
}
#endif

/*!
 * @brief This internal API is used to compensate the raw humidity data and
 * return the compensated humidity data in integer data type.
 */
uint32_t BME280::CompensateHumidity(const struct bme280_uncomp_data *uncomp_data)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	uint32_t humidity;
	uint32_t humidity_max = 102400;

	var1 = CalibData->t_fine - ((int32_t)76800);
	var2 = (int32_t)(uncomp_data->humidity * 16384);
	var3 = (int32_t)(((int32_t)CalibData->dig_H4) * 1048576);
	var4 = ((int32_t)CalibData->dig_H5) * var1;
	var5 = (((var2 - var3) - var4) + (int32_t)16384) / 32768;
	var2 = (var1 * ((int32_t)CalibData->dig_H6)) / 1024;
	var3 = (var1 * ((int32_t)CalibData->dig_H3)) / 2048;
	var4 = ((var2 * (var3 + (int32_t)32768)) / 1024) + (int32_t)2097152;
	var2 = ((var4 * ((int32_t)CalibData->dig_H2)) + 8192) / 16384;
	var3 = var5 * var2;
	var4 = ((var3 / 32768) * (var3 / 32768)) / 128;
	var5 = var3 - ((var4 * ((int32_t)CalibData->dig_H1)) / 16);
	var5 = (var5 < 0 ? 0 : var5);
	var5 = (var5 > 419430400 ? 419430400 : var5);
	humidity = (uint32_t)(var5 / 4096);

	if (humidity > humidity_max)
		humidity = humidity_max;

	return humidity;
}
#endif

/****************************************************************/
bool BME280::ReadTrim()
{
    uint8_t ord = 0;
    uint8_t dig[32] = {0,};
    bool success = true;

    // Temp. Dig
    success &= IFace->ReadBuffer(TEMP_DIG_ADDR, &dig[ord], TEMP_DIG_LENGTH);
    ord += TEMP_DIG_LENGTH;

    // Pressure Dig
    success &= IFace->ReadBuffer(PRESS_DIG_ADDR, &dig[ord], PRESS_DIG_LENGTH);
    ord += PRESS_DIG_LENGTH;

    // Humidity Dig 1
    success &= IFace->ReadBuffer(HUM_DIG_ADDR1, &dig[ord], HUM_DIG_ADDR1_LENGTH);
    ord += HUM_DIG_ADDR1_LENGTH;

    // Humidity Dig 2
    success &= IFace->ReadBuffer(HUM_DIG_ADDR2, &dig[ord], HUM_DIG_ADDR2_LENGTH);
    ord += HUM_DIG_ADDR2_LENGTH;

    CalibData->dig_T1 = BME280_CONCAT_BYTES(dig[1], dig[0]);
    CalibData->dig_T2 = (int16_t)BME280_CONCAT_BYTES(dig[3], dig[2]);
    CalibData->dig_T3 = (int16_t)BME280_CONCAT_BYTES(dig[5], dig[4]);
    CalibData->dig_P1 = BME280_CONCAT_BYTES(dig[7], dig[6]);
    CalibData->dig_P2 = (int16_t)BME280_CONCAT_BYTES(dig[9], dig[8]);
    CalibData->dig_P3 = (int16_t)BME280_CONCAT_BYTES(dig[11], dig[10]);
    CalibData->dig_P4 = (int16_t)BME280_CONCAT_BYTES(dig[13], dig[12]);
    CalibData->dig_P5 = (int16_t)BME280_CONCAT_BYTES(dig[15], dig[14]);
    CalibData->dig_P6 = (int16_t)BME280_CONCAT_BYTES(dig[17], dig[16]);
    CalibData->dig_P7 = (int16_t)BME280_CONCAT_BYTES(dig[19], dig[18]);
    CalibData->dig_P8 = (int16_t)BME280_CONCAT_BYTES(dig[21], dig[20]);
    CalibData->dig_P9 = (int16_t)BME280_CONCAT_BYTES(dig[23], dig[22]);
    CalibData->dig_H1 = dig[25];

#ifdef LOGGING
   hex_dump(dig, ord, 8, "ReadTrim");
#endif

    return success && ord == DIG_LENGTH;
}


/*!
 * @brief This internal API reads the calibration data from the sensor, parse
 * it and store in the device structure.
 */
bool BME280::GetCalibData()
{
	bool rslt;

	uint8_t reg_addr = BME280_TEMP_PRESS_CALIB_DATA_ADDR;
	// Array to store calibration data 
	uint8_t calib_data[BME280_TEMP_PRESS_CALIB_DATA_LEN] = {0};

#ifdef LOGGING
	Log->Msg((char*)"BME280::GetCalibData");
#endif

	// Read the calibration data from the sensor 
    rslt = IFace->ReadBuffer(reg_addr, calib_data, BME280_TEMP_PRESS_CALIB_DATA_LEN);

	if (rslt) {
		// Parse temperature and pressure calibration data and store it in device structure 
		ParseTempCalibData(calib_data);
		reg_addr = BME280_HUMIDITY_CALIB_DATA_ADDR;

        // Read the humidity calibration data from the sensor
        rslt = IFace->ReadBuffer(reg_addr, calib_data, BME280_HUMIDITY_CALIB_DATA_LEN);
		if (rslt) {
			// Parse humidity calibration data and store it in device structure 
			ParseHumidityCalibData(calib_data);
		}
	}

	return rslt;
}

/*!
 *  @brief This internal API is used to parse the temperature and
 *  pressure calibration data and store it in device structure.
 */
void BME280::ParseTempCalibData(const uint8_t *reg_data)
{

#ifdef LOGGING
	Log->Msg((char*)"BME280::ParseTempCalibData");
#endif

	CalibData->dig_T1 = BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	CalibData->dig_T2 = (int16_t)BME280_CONCAT_BYTES(reg_data[3], reg_data[2]);
	CalibData->dig_T3 = (int16_t)BME280_CONCAT_BYTES(reg_data[5], reg_data[4]);
	CalibData->dig_P1 = BME280_CONCAT_BYTES(reg_data[7], reg_data[6]);
	CalibData->dig_P2 = (int16_t)BME280_CONCAT_BYTES(reg_data[9], reg_data[8]);
	CalibData->dig_P3 = (int16_t)BME280_CONCAT_BYTES(reg_data[11], reg_data[10]);
	CalibData->dig_P4 = (int16_t)BME280_CONCAT_BYTES(reg_data[13], reg_data[12]);
	CalibData->dig_P5 = (int16_t)BME280_CONCAT_BYTES(reg_data[15], reg_data[14]);
	CalibData->dig_P6 = (int16_t)BME280_CONCAT_BYTES(reg_data[17], reg_data[16]);
	CalibData->dig_P7 = (int16_t)BME280_CONCAT_BYTES(reg_data[19], reg_data[18]);
	CalibData->dig_P8 = (int16_t)BME280_CONCAT_BYTES(reg_data[21], reg_data[20]);
	CalibData->dig_P9 = (int16_t)BME280_CONCAT_BYTES(reg_data[23], reg_data[22]);
	CalibData->dig_H1 = reg_data[25];
}

/*!
 *  @brief This internal API is used to parse the humidity calibration data
 *  and store it in device structure.
 */
void BME280::ParseHumidityCalibData(const uint8_t *reg_data)
{
#ifdef LOGGING
	Log->Msg((char*)"BME280::ParseHumidityCalibData");
#endif
	int16_t dig_H4_lsb;
	int16_t dig_H4_msb;
	int16_t dig_H5_lsb;
	int16_t dig_H5_msb;

	CalibData->dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
	CalibData->dig_H3 = reg_data[2];

	dig_H4_msb = (int16_t)(int8_t)reg_data[3] * 16;
	dig_H4_lsb = (int16_t)(reg_data[4] & 0x0F);
	CalibData->dig_H4 = dig_H4_msb | dig_H4_lsb;

	dig_H5_msb = (int16_t)(int8_t)reg_data[5] * 16;
	dig_H5_lsb = (int16_t)(reg_data[4] >> 4);
	CalibData->dig_H5 = dig_H5_msb | dig_H5_lsb;
	CalibData->dig_H6 = (int8_t)reg_data[6];
}

/*!
 * @brief This internal API is used to identify the settings which the user
 * wants to modify in the sensor.
 */
uint8_t BME280::AreSettingsChanged(uint8_t sub_settings, uint8_t desired_settings)
{
	uint8_t settings_changed = false;

	if (sub_settings & desired_settings) {
		/* User wants to modify this particular settings */
		settings_changed = true;
	} else {
		/* User don't want to modify this particular settings */
		settings_changed = false;
	}

	return settings_changed;
}
