#include <iostream>
#include <memory>
#include <unistd.h>

#include "bme280.h"
#include "logger.h"

using namespace std;


int main()
{
    TData sensorData;
    // logger initialization
    Logger *logger = Logger::GetInstance();
    logger->Msg("MAIN::BME280 test app");

    // bme280 initialization
    shared_ptr<BME280> sen = make_shared<BME280>();
    sen->Init("/dev/i2c-2");

    // bme280 settings
    uint8_t settings_sel;
    settings_sel = BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL;
    sen->SetSensorSettings(settings_sel);
    
    //while(true) {
    for(int i=0; i<3; i++) {
        sen->SetSensorMode(BME280_FORCED_MODE);
        // Wait for the measurement to complete and print data @25Hz 
	    usleep(1000*40);
#ifdef ALT_MEAS
        string temp = string("MAIN::Temp: ") + to_string(sen->GetTemperature());
        string pres = string("MAIN::Pres: ") + to_string(sen->GetPressure());
        string hum = string("MAIN::Hum : ") + to_string(sen->GetHumidity());
#else
        sen->GetSensorData(BME280_ALL);
        
        string temp = string("MAIN::Temp: ") + to_string(sen->GetTemperature());
        //string pres = string("MAIN::Pres: ") + to_string(sen->GetPressure());
        //string hum = string("MAIN::Hum : ") + to_string(sen->GetHumidity());
#endif
        logger->Msg((char*)temp.c_str());
        //logger->Msg((char*)pres.c_str());
        //logger->Msg((char*)hum.c_str());
        sleep(3);
    }
    
    return EXIT_SUCCESS;
}
