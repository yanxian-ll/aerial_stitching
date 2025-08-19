#include "debug_utils.h"

DebugLogger::DebugLogger() {
    std::filesystem::create_directories("debug");

    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);

    std::ostringstream filename;
    filename << "debug/log_" << std::put_time(&now_tm, "%Y%m%d_%H%M%S") << ".txt";
    setLogFile(filename.str());
}

DebugLogger::~DebugLogger() {
    if (logFile.is_open()) {
        logFile.flush();
        logFile.close();
    }
}

void DebugLogger::setLogFile(const std::string& filename) {
    std::lock_guard<std::mutex> lock(logMutex);
    if (logFile.is_open()) {
        logFile.close();
    }

    // 获取当前时间戳
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);

    std::ostringstream ts;
    ts << "_" << std::put_time(&now_tm, "%Y%m%d_%H%M%S");

    // 拆分文件名（不含扩展名）
    std::string finalName = filename;
    auto pos = filename.find_last_of('.');
    if (pos != std::string::npos) {
        finalName = filename.substr(0, pos) + ts.str() + filename.substr(pos);
    } else {
        finalName = filename + ts.str() + ".log";
    }

    currentLogFile = finalName;
    logFile.open(finalName, std::ios::app);

    if (!logFile.is_open()) {
        std::cerr << "Error: Could not open log file: " << finalName << std::endl;
    }
}

void DebugLogger::log(const std::string& message, LogLevel level) {
    auto now = std::chrono::system_clock::now();
    auto now_time = std::chrono::system_clock::to_time_t(now);
    std::tm now_tm = *std::localtime(&now_time);

    std::ostringstream logLine;
    logLine << "[" << std::put_time(&now_tm, "%Y-%m-%d %H:%M:%S") << "][" 
            << levelToStr(level) << "] " << message << "\n";

    std::lock_guard<std::mutex> lock(logMutex);
    if (logFile.is_open()) 
    {
        logFile << logLine.str();
        logFile.flush();
    }
    if (consoleOutput) std::cout << logLine.str();
}

