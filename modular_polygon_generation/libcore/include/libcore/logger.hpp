#pragma once

#include <iostream>
#include <sstream>
#include <mutex>
#include <string>
#include <iomanip>
#include <chrono>
#include <ctime>

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
        if (!prefixPrinted_) {
            std::cerr << getColorCode() << "[" << getTimestamp() << "] [" << levelToString() << "] ";
            prefixPrinted_ = true;
        }
        std::cerr << val;
        return *this;
    }

    LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::cerr << manip;
        if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl)) {
            std::cerr << "\033[0m";  // Reset color
            prefixPrinted_ = false; // Next message will get prefix again
        }
        return *this;
    }

private:
    Level level_;
    bool prefixPrinted_ = false;
    std::mutex mutex_;

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
