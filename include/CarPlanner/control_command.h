#pragma once
#include <CarPlanner/ninjacar.h>

/// Structure to hold the steering, acceleration and reaction wheel commands
/// that are sent to the vehicle.
class ControlCommand
{
public:
    ControlCommand(): force_(0), curvature_(0), m_dT(0),m_dPhi(0), torque_(0,0,0),timestamp_(0)
    {

    }
    ControlCommand(const double& force,const double& curvature,
                    const Eigen::Vector3d& torques, const double& dt, const double& dPhi){

        force_ = force;
        //m_dControlAccel = accel;
        curvature_ = curvature;
        torque_ = torques;
        m_dT = dt;
        m_dPhi = dPhi;
    }

    //double m_dControlAccel;
    double force_;
    double curvature_;
    double m_dT;
    double m_dPhi;
    Eigen::Vector3d torque_;
    double timestamp_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};

/// List holding the previous commands sent to the model
/// (Newest commands at the front, oldest at the back).
typedef std::list<ControlCommand> CommandList;
