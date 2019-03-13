#include "logger.h"
#include <stdarg.h>
#include <iostream>

using namespace std::chrono;

Logger *Logger::theInstance = NULL;

Logger* Logger::GetInstance()
{
	if (theInstance == NULL)
	{
		theInstance = new Logger();
	}
	return theInstance;
}

Logger::Logger()
{
#ifdef FILE_LOGGING        
	logFile = fopen("Log.txt", "wt");
#endif    
}
 
Logger::~Logger()
{
#ifdef FILE_LOGGING
	if (logFile)
	{
		fflush(logFile);
		fclose(logFile);
		logFile = NULL;
	}
#endif
    this->Release();
}
 
void Logger::Release()
{
	if (theInstance != NULL)
	{
		delete theInstance;
	}
	theInstance = NULL;
}
 
bool Logger::Msg(char* _log, ...)
{
	char acText[1024];
	va_list VAList;
 
	va_start(VAList, _log);
	vsprintf(acText, _log, VAList);
	va_end(VAList);

    
#ifdef FILE_LOGGING
	fprintf(Logger::logFile, "%s (Line %d): %s \n",  __FILE__, __LINE__, acText);
#endif
    //std::cout << __FILE__ << ":" << __LINE__ << "\t" << acText << std::endl;
    std::cout << this->GetTimestamp().count() << ": " << acText << std::endl;
 
	return true;
}

milliseconds Logger::GetTimestamp()
{
    milliseconds ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return ms;
}

#if 0
#include "logger.h"
#include <iostream>


Logger::Logger(const char *tag)
{
    Tag = std::string(tag);
}

void Logger::PrintMsg(std::string msg)
{
    std::cout << Tag << ": " << msg << std::endl;
}

#endif