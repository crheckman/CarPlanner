#pragma once
#include <CarPlanner/ninjacar.h>
#include <CarPlanner/ninjacar_impl.h>

namespace carplanner {

struct RegressionParameter
{
    RegressionParameter(std::map<int,  double>& map,
                        int nKey,
                        std::shared_ptr<NinjaCar<Vehicle>> vehicle = NULL);

    static bool AreEqual(const std::vector<RegressionParameter>& params1,
                         const std::vector<RegressionParameter>& params2);
    void UpdateValue(const double newVal);
    double val_;
    int key_;
    std::string name_;
    std::shared_ptr<NinjaCar<Vehicle>> vehicle_;
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

} //namespace carplanner
