#pragma once

#include <CarPlanner/ninjacar.h>

enum ControlTarget
{
    eTargetSimulation = 0,
    eTargetExperiment = 1
};


/// Structure containing all parameters for a car
class VehicleParameters
{
public:

  static const char * const Names[];
  static bool LoadFromFile(const std::string sFile, std::map<int, double> &map);
  static void PrintAllParams(const std::map<int, double> &map);
  static bool SaveToFile(const std::string sFile, const std::map<int, double> &map);

};
