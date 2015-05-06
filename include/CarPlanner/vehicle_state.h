#pragma once
#include <CarPlanner/ninjacar.h>

struct VehicleState
{
    VehicleState()
    {
        m_dTwv = Sophus::SE3d();
        m_dV.setZero();
        m_dW.setZero();
        m_dSteering = 0;
        m_dCurvature = 0 ;
        m_dTime = 0;
    }

    VehicleState(const Sophus::SE3d& dTwv, const double dV, double dCurvature = 0)
    {
        m_dCurvature = dCurvature;
        m_dTwv = dTwv;
        Eigen::Vector3d vecV;
        vecV << dV,0,0;
        m_dV = m_dTwv.so3()*vecV;
        m_dW << 0,0,0;
        m_dSteering = 0;
    }

    static VehicleState GetInterpolatedState(const std::vector<VehicleState>& vStates,
                                             const int nStartIndex,
                                             const double& time,
                                             int& nIndex)
    {
        VehicleState stateOut = vStates.back();
        double currTime = vStates.front().m_dTime;
        for( size_t ii = nStartIndex; ii < vStates.size()-1 ; ii++){
            nIndex = ii;
            if(vStates[ii+1].m_dTime >= time){
                double interpolation = (time-currTime)/(vStates[ii+1].m_dTime-currTime);
                const VehicleState& state2 = vStates[ii+1];
                const VehicleState& state1 = vStates[ii];
                stateOut.m_dCurvature = interpolation*state2.m_dCurvature + (1-interpolation)*state1.m_dCurvature;
                stateOut.m_dSteering = interpolation*state2.m_dSteering + (1-interpolation)*state1.m_dSteering;
                stateOut.m_dV = interpolation*state2.m_dV + (1-interpolation)*state1.m_dV;
                stateOut.m_dW = interpolation*state2.m_dW + (1-interpolation)*state1.m_dW;
                Eigen::Vector3d trans = interpolation*state2.m_dTwv.translation() + (1-interpolation)*state1.m_dTwv.translation();
                Eigen::Quaterniond rot = state1.m_dTwv.so3().unit_quaternion().slerp(interpolation,state2.m_dTwv.so3().unit_quaternion());
                stateOut.m_dTwv = Sophus::SE3d(rot,trans);
                break;
            }else{
                currTime = vStates[ii+1].m_dTime;
            }
        }
        return stateOut;
    }

    void UpdateWheels(const std::vector<Sophus::SE3d>& vWheelTransforms){
        Sophus::SE3d bodyT = m_dTwv;
        m_vWheelContacts.resize(4);
        m_vWheelStates.resize(4);
        for(size_t ii = 0 ; ii < m_vWheelContacts.size() ; ii++){
            //position the wheels with respect to the body
            fflush(stdout);
            Sophus::SE3d T = bodyT* vWheelTransforms[ii];
            m_vWheelStates[ii] = T;
        }
    }

    static void AlignWithVelocityVector(Sophus::SE3d& Twv, const Eigen::Vector3d& dV)
    {
        Eigen::Vector3d vel = dV.normalized();
        Eigen::Matrix3d R = Twv.so3().matrix();
        R.block<3,1>(0,0) = vel;
        R.block<3,1>(0,2) = R.block<3,1>(0,0).cross(R.block<3,1>(0,1)).normalized();
        R.block<3,1>(0,1) = R.block<3,1>(0,2).cross(R.block<3,1>(0,0)).normalized();
        Twv.so3() = Sophus::SO3d(R);
    }

    void AlignWithVelocityVector()
    {
        VehicleState::AlignWithVelocityVector(m_dTwv,m_dV);
    }

    /// Checks all ground contact points, and returns true if all wheels are off the ground
    bool IsAirborne() const
    {
        bool ground = false;
        if(m_vWheelContacts.size() == 4 )
        {
            for(unsigned int ii = 0 ; ii < 4 ; ii++){
                if(m_vWheelContacts[ii] == true) {
                    ground = true;
                }
            }
        }
        return !ground;
    }

    /// Uses the VehicleStateToPose function to convert the
    Eigen::Vector6d ToPose(){
        return VehicleStateToPose(*this);
    }

    double GetTheta() const
    {
        //calculate theta
        Eigen::Vector3d down,right,forward, trueForward;
        down << 0,0,1;
        trueForward << 1,0,0;
        forward = GetBasisVector(m_dTwv,0); //get the forward
        return atan2(forward[1],forward[0]);
    }

    /// This function gets a vehicle state and returns a 5d pose vector which is parametrized as
    /// [x,y,t,k,v] where t is the 2d angle, k is the path curvature and v is the velocity
    static Eigen::Vector6d VehicleStateToPose(const VehicleState& state //< The given state to construct the pose from
                                              )
    {
        Eigen::Vector6d poseOut;
        poseOut << state.m_dTwv.translation()[0],
                   state.m_dTwv.translation()[1],
                   state.m_dTwv.translation()[2],
                   state.GetTheta(),
                   state.m_dCurvature,
                   state.m_dV.norm();
        return poseOut;
    }



    Sophus::SE3d m_dTwv;                     //< 4x4 matrix denoting the state of the car
    std::vector<Sophus::SE3d> m_vWheelStates;   //< 4x4 matrices which denote the pose of each wheel
    std::vector<bool> m_vWheelContacts;         //< Angular velocity of the vehicle in world coordinates

    Eigen::Vector3d m_dV;                       //< Linear velocity of the vehicle in world coordinates
    Eigen::Vector3d m_dW;                       //< Angular velocity of the vehicle in world coordinates

    double m_dCurvature;                        //< The curvature at this point in the path
    double m_dSteering;                         //< The steering command given to the car at this point (used to reset rate limitations)
    double m_dTime;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
