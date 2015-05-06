#pragma once
#include <CarPlanner/ninjacar.h>
#include <CarPlanner/ninjacar_impl.h>

namespace carplanner {

struct RegressionParameter
{
    RegressionParameter(std::map<int,  double>& map,
                        int nKey,
                        std::shared_ptr<NinjaCar<Vehicle,Controller,Planner>> pModel = NULL);

    static bool AreEqual(const std::vector<RegressionParameter>& params1,
                         const std::vector<RegressionParameter>& params2);
    void UpdateValue(const double newVal);
    double vel_w_dot_al;
    int m_nKey;
    std::string m_sName;
    std::shared_ptr<NinjaCar<Vehicle,Controller,Planner>> m_pModel;
};

/// Redefinition of redirection operators.
inline std::ostream& operator<<( std::ostream& Stream,
                                 RegressionParameter& parameter )
{
    Stream << parameter.vel_w_dot_al;
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
        Stream << params[ii].m_sName << ":";
        Stream << params[ii].vel_w_dot_al;
        Stream << ", ";
    }

    Stream << " ]";

    return Stream;

}

} //namespace carplanner
