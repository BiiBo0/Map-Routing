#ifndef UTILS_PROFILER_H
#define UTILS_PROFILER_H

#include <iostream>
#include <string>
#include <chrono>

using Clock = std::chrono::high_resolution_clock;

namespace ProfilerColors {
    constexpr const char* BOLD_YELLOW = "\x1B[1;33m";
    constexpr const char* RESET       = "\x1B[0m";
}

class Profiler {
public:
    Profiler(const std::string& name)
        : m_name(name), m_startTime(Clock::now()), m_stopped(false)
    {
    }

    ~Profiler() noexcept {
        try {
            if (!m_stopped) {
                stop(); 
            }
        } catch (const std::exception& e) {
            std::cerr << "[Profiler] Exception in destructor for '" << m_name << "': " << e.what() << std::endl;
        } catch (...) {
            std::cerr << "[Profiler] Unknown exception in destructor for '" << m_name << "'" << std::endl;
        }
    }

    double stop() {
        if (m_stopped)
            return m_durationMs;

        auto endTime = Clock::now();
        auto duration = endTime - m_startTime;
        long long us = std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
        m_durationMs = us / 1000.0;
        m_stopped = true;

        return m_durationMs;
    }

    Profiler(const Profiler&) = delete;
    Profiler& operator=(const Profiler&) = delete;
    Profiler(Profiler&&) = delete;
    Profiler& operator=(Profiler&&) = delete;

private:
    std::string m_name;
    std::chrono::time_point<Clock> m_startTime;
    bool m_stopped;
    double m_durationMs = 0.0;
};

#endif // UTILS_PROFILER_H
