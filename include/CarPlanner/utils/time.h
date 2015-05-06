#pragma once
#include <stdio.h>
#include <fstream>
#include <vector>
#include <sys/time.h>
#include <time.h>
#include <chrono>
#include <mach/clock.h>
#include <mach/mach.h>

inline void current_utc_time(struct timespec *ts) {

#ifdef __MACH__ // OS X does not have clock_gettime, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ts->tv_sec = mts.tv_sec;
  ts->tv_nsec = mts.tv_nsec;
#else
  clock_gettime(CLOCK_REALTIME, ts);
#endif

}

namespace CarPlanner {

// Aux Time Functions
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double Tic() {

  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_sec  + 1e-6 * (tv.tv_usec);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double RealTime() {
  return Tic();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline double Toc(double dTic) {
  return Tic() - dTic;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline double TocMS(double dTic) {
  return ( Tic() - dTic)*1000.;
}


} // end namespace CarPlanner
