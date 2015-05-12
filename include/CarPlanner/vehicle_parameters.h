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

struct RegressionParameter
{
  RegressionParameter(std::map<int, double>& map,
                      int nKey,
                      std::shared_ptr<carplanner::NinjaCar> vehicle );

  static bool AreEqual(const std::vector<RegressionParameter>& params1,
                       const std::vector<RegressionParameter>& params2);
  void UpdateValue(const double newVal);
  double val_;
  int key_;
  std::string name_;
  std::shared_ptr<carplanner::NinjaCar> vehicle_;
};

/// Redefinition of redirection operators.
inline std::ostream& operator<<( std::ostream& Stream,
                                 RegressionParameter& parameter )
{
  Stream << parameter.val_;
  return Stream;
}

inline std::istream& operator>>( std::istream& Stream,
                                 RegressionParameter& parameter )
{
  double val;
  Stream >> val;
  parameter.UpdateValue(val);
  return Stream;
}

inline std::ostream& operator<<( std::ostream& Stream,
                                 const std::vector<RegressionParameter>& params )
{
  Stream << "[ ";

  for( unsigned int ii = 0; ii < params.size(); ii++ ) {
    Stream << params[ii].name_ << ":";
    Stream << params[ii].val_;
    Stream << ", ";
  }

  Stream << " ]";

  return Stream;

}
