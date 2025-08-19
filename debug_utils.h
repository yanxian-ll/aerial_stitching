#ifndef DEBUG_UTILES_H
#define DEBUG_UTILES_H

#include <string>
#include <fstream>
#include <ctime>
#include <mutex>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <filesystem>

enum class LogLevel { DEBUG, INFO, WARN, ERROR };

class DebugLogger {
public:
    static DebugLogger& getInstance() {
        static DebugLogger instance;
        return instance;
    }

    void log(const std::string& message, LogLevel level = LogLevel::DEBUG);
    void setLogFile(const std::string& filename);
    void enableConsoleOutput(bool enable) { consoleOutput = enable; }

private:
    DebugLogger();
    ~DebugLogger();

    std::ofstream logFile;
    std::mutex logMutex;
    std::string currentLogFile;
    bool consoleOutput = true;

    const char* levelToStr(LogLevel level) {
        switch(level) {
            case LogLevel::DEBUG: return "DEBUG";
            case LogLevel::INFO:  return "INFO";
            case LogLevel::WARN:  return "WARN";
            case LogLevel::ERROR: return "ERROR";
            default: return "UNKNOWN";
        }
    }
};


#ifdef ENABLE_DEBUG_LOG
    #define LOG_DEBUG(msg) DebugLogger::getInstance().log(msg, LogLevel::DEBUG)
#else
    #define LOG_DEBUG(msg)  ((void)0)   // 空操作
#endif

#define LOG_INFO(msg)  DebugLogger::getInstance().log(msg, LogLevel::INFO)
#define LOG_WARN(msg)  DebugLogger::getInstance().log(msg, LogLevel::WARN)
#define LOG_ERROR(msg) DebugLogger::getInstance().log(msg, LogLevel::ERROR)

#endif  // DEBUG_UTILES_H
