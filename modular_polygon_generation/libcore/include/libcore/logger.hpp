#pragma once

#include <iostream>
#include <sstream>
#include <fstream>
#include <mutex>
#include <string>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <filesystem>

namespace libcore {
class LogStream {
public:
    enum class Level {
        DEBUG, INFO, WARNING, ERROR
    };

    LogStream(Level level) : level_(level) {}

    template<typename T>
    LogStream& operator<<(const T& val) {
        std::lock_guard<std::mutex> lock(mutex_);
        ensureLogFileOpen();

        if (!prefixPrinted_) {
            std::string prefix = "[" + getTimestamp() + "] [" + levelToString() + "] ";
            std::cerr << getColorCode() << prefix;
            logfile_ << prefix;
            prefixPrinted_ = true;
        }

        std::cerr << val;
        logfile_ << val;
        return *this;
    }

    LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cerr << manip;
        logfile_ << manip;
        if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl)) {
            std::cerr << "\033[0m";  // Reset color
            prefixPrinted_ = false;
        }
        return *this;
    }

private:
    Level level_;
    bool prefixPrinted_ = false;
    std::mutex mutex_;

    static inline std::ofstream logfile_;
    static inline std::once_flag init_flag_;
    static inline std::string logfile_path_;

    void ensureLogFileOpen() {
        std::call_once(init_flag_, []() {
            // Create log directory
            std::filesystem::create_directories("log");

            // Generate filename: log/YYYY-MM-DD_HH-MM-SS.log
            auto now = std::chrono::system_clock::now();
            auto itt = std::chrono::system_clock::to_time_t(now);
            std::ostringstream ss;
            ss << "log/" << std::put_time(std::localtime(&itt), "%Y-%m-%d_%H-%M-%S") << ".log";
            logfile_path_ = ss.str();

            // Open log file
            logfile_.open(logfile_path_, std::ios::app);
        });
    }

    std::string levelToString() const {
        switch (level_) {
            case Level::DEBUG: return "Debug";
            case Level::INFO: return "Info";
            case Level::WARNING: return "Warning";
            case Level::ERROR: return "Error";
        }
        return "Unknown";
    }

    std::string getColorCode() const {
        switch (level_) {
            case Level::DEBUG: return "\033[1;36m";  // Cyan
            case Level::INFO: return "\033[1;32m";   // Green
            case Level::WARNING: return "\033[1;33m";// Yellow
            case Level::ERROR: return "\033[1;31m";  // Red
        }
        return "\033[0m";
    }

    std::string getTimestamp() const {
        auto now = std::chrono::system_clock::now();
        auto itt = std::chrono::system_clock::to_time_t(now);
        std::ostringstream ss;
        ss << std::put_time(std::localtime(&itt), "%H:%M:%S");
        return ss.str();
    }
};

// Global log stream instances
inline LogStream debug(LogStream::Level::DEBUG);
inline LogStream info(LogStream::Level::INFO);
inline LogStream warning(LogStream::Level::WARNING);
inline LogStream error(LogStream::Level::ERROR);
} // namespace libcore
