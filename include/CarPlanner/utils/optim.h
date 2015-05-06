#pragma once
#include <thread>
#include <mutex>
#include <atomic>

#ifdef __APPLE__
#define SetThreadName(x) pthread_setname_np(x);
#else
#include <sys/prctl.h>
#define SetThreadName(x) prctl(PR_SET_NAME,x,0,0,0);
#endif

#define DEBUG 1
#ifdef DEBUG
#define dout(str) std::cout << __FUNCTION__ << " --  " << str << std::endl
#define dout_cond(str,x) if(x)std::cout << __FUNCTION__ << " --  " << str << std::endl
#else
#define dout(str)
#endif

//#define CAR_HEIGHT_OFFSET 0.06

enum OptimizationTask {
  eGaussNewton = 0,
  eDiscrete = 1,
  eDiscreteSearch = 2
};

enum PpmChannels
{
  ePpm_Accel = 3,
  ePpm_Steering = 2
};

enum WaypointType
{
  eWaypoint_Normal = 1,
  eWayoiint_FlipFront = 2
};

enum eLocType { VT_AIR, VT_GROUND, VT_REC_RAMP, VT_CIR_RAMP };

