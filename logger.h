#ifndef __LOGGER_H
#define __LOGGER_H

#include <string>

#if 0
class Logger
{
    public:
        Logger(const char *);
        void PrintMsg(std::string msg);

    private:
        std::string Tag;  
};
#endif

#include <cstdio>
#include <chrono>

using namespace std::chrono;

class Logger 
{
	private:
		Logger();
		~Logger();
        milliseconds GetTimestamp();
#ifdef FILE_LOGGING
		FILE*       logFile;
#endif
        std::string Tag;  

		static Logger *theInstance;

	public:
		static Logger   *GetInstance();
		static void     Release();
		bool            Msg(char* _log, ...);
};

#endif