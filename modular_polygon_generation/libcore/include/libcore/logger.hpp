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

namespace logger {

class LogStream {
public:
    enum class Level { DEBUG, INFO, WARNING, ERROR };

    LogStream(Level level) : level_(level) {}

    // Configure the log folder before first log
    static void setLogFolder(const std::filesystem::path& folder) {
        log_dir_ = folder;
    }

    // Optional: set a fixed base name (no extension)
    static void setBaseName(const std::string& base) {
        base_name_ = base;
    }

    template<typename T>
    LogStream& operator<<(const T& val) {
        std::lock_guard<std::mutex> lock(global_mutex_);
        ensureLogFileOpen_();

        if (!prefixPrinted_) {
            std::string prefix = "[" + getTimestamp_() + "] [" + levelToString_() + "] ";
            std::cerr << getColorCode_() << prefix;
            logfile_ << prefix;
            prefixPrinted_ = true;
        }

        std::cerr << val;
        logfile_ << val;
        return *this;
    }

    LogStream& operator<<(std::ostream& (*manip)(std::ostream&)) {
        std::lock_guard<std::mutex> lock(global_mutex_);
        ensureLogFileOpen_();

        std::cerr << manip;
        logfile_ << manip;
        if (manip == static_cast<std::ostream& (*)(std::ostream&)>(std::endl)) {
            std::cerr << "\033[0m";  // Reset color
            prefixPrinted_ = false;
            logfile_.flush();
        }
        return *this;
    }

private:
    Level level_;
    bool prefixPrinted_ = false;

    // ---- Shared static state ----
    static inline std::mutex        global_mutex_;     // guards init + writes
    static inline std::ofstream     logfile_;
    static inline bool              initialized_ = false;
    static inline std::filesystem::path log_dir_;
    static inline std::string       base_name_ = "";   // if empty -> timestamped filename
    static inline std::filesystem::path logfile_fullpath_;

    void ensureLogFileOpen_() {
        if (initialized_ && logfile_.is_open()) return;

        if (log_dir_.empty()) {
            std::cerr << "Log directory not set. Using default directory." << std::endl;
            log_dir_ = std::filesystem::current_path() / "logs";
        }

        // Make sure directory exists
        std::error_code ec;
        std::filesystem::create_directories(log_dir_, ec);

        // Build filename
        if (base_name_.empty()) {
            auto now = std::chrono::system_clock::now();
            auto itt = std::chrono::system_clock::to_time_t(now);
            std::ostringstream ss;
            ss << std::put_time(std::localtime(&itt), "%Y-%m-%d_%H-%M-%S");
            base_name_ = ss.str();
            logfile_fullpath_ = log_dir_ / (base_name_ + ".log");
        } else {
            logfile_fullpath_ = log_dir_ / (base_name_ + ".log");
        }

        logfile_.open(logfile_fullpath_, std::ios::app);
        initialized_ = true;
    }

    std::string levelToString_() const {
        switch (level_) {
            case Level::DEBUG:   return "Debug";
            case Level::INFO:    return "Info";
            case Level::WARNING: return "Warning";
            case Level::ERROR:   return "Error";
        }
        return "Unknown";
    }

    std::string getColorCode_() const {
        switch (level_) {
            case Level::DEBUG:   return "\033[1;36m";
            case Level::INFO:    return "\033[1;32m";
            case Level::WARNING: return "\033[1;33m";
            case Level::ERROR:   return "\033[1;31m";
        }
        return "\033[0m";
    }

    static std::string getTimestamp_() {
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

// Convenience free functions
inline void setLogFolder(const std::filesystem::path& folder) {
    LogStream::setLogFolder(folder);
}
inline void setLogBaseName(const std::string& base) {
    LogStream::setBaseName(base);
}

} // namespace logger
