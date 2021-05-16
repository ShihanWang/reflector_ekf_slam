

#include "common/time.h"

#include <time.h>

#include <cerrno>
#include <cstring>
#include <string>

#include "glog/logging.h"

namespace common {

Duration FromSeconds(const double seconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::duration<double>(seconds));
}

double ToSeconds(const Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

double ToSeconds(const std::chrono::steady_clock::duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<double>>(duration)
      .count();
}

Time FromUniversal(const int64 ticks) { return Time(Duration(ticks)); }

int64 ToUniversal(const Time time) { return time.time_since_epoch().count(); }

std::ostream& operator<<(std::ostream& os, const Time time) {
  os << std::to_string(ToUniversal(time));
  return os;
}

common::Duration FromMilliseconds(const int64 milliseconds) {
  return std::chrono::duration_cast<Duration>(
      std::chrono::milliseconds(milliseconds));
}

double GetThreadCpuTimeSeconds() {
#ifndef WIN32
  struct timespec thread_cpu_time;
  CHECK(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &thread_cpu_time) == 0)
      << std::strerror(errno);
  return thread_cpu_time.tv_sec + 1e-9 * thread_cpu_time.tv_nsec;
#else
  return 0.;
#endif
}

}  // namespace common
