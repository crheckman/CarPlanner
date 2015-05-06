#pragma once
#include <CarPlanner/ninjacar.h>

struct VehicleState
{
    VehicleState()
    {
        t_wv_ = Sophus::SE3d();
        vel_w_dot_.setZero();
        omega_w_dot_.setZero();
        steering_cmd_ = 0;
        curvature_ = 0 ;
        timestamp_ = 0;
    }

    VehicleState(const Sophus::SE3d& dTwv, const double dV, double dCurvature = 0)
    {
        curvature_ = dCurvature;
        t_wv_ = dTwv;
        Eigen::Vector3d vecV;
        vecV << dV,0,0;
        vel_w_dot_ = t_wv_.so3()*vecV;
        omega_w_dot_ << 0,0,0;
        steering_cmd_ = 0;
    }

    static VehicleState GetInterpolatedState(const std::vector<VehicleState>& vStates,
                                             const int nStartIndex,
                                             const double& time,
                                             int& nIndex)
    {
        VehicleState stateOut = vStates.back();
        double currTime = vStates.front().timestamp_;
        for( size_t ii = nStartIndex; ii < vStates.size()-1 ; ii++){
            nIndex = ii;
            if(vStates[ii+1].timestamp_ >= time){
                double interpolation = (time-currTime)/(vStates[ii+1].timestamp_-currTime);
                const VehicleState& state2 = vStates[ii+1];
                const VehicleState& state1 = vStates[ii];
                stateOut.curvature_ = interpolation*state2.curvature_ + (1-interpolation)*state1.curvature_;
                stateOut.steering_cmd_ = interpolation*state2.steering_cmd_ + (1-interpolation)*state1.steering_cmd_;
                stateOut.vel_w_dot_ = interpolation*state2.vel_w_dot_ + (1-interpolation)*state1.vel_w_dot_;
                stateOut.omega_w_dot_ = interpolation*state2.omega_w_dot_ + (1-interpolation)*state1.omega_w_dot_;
                Eigen::Vector3d trans = interpolation*state2.t_wv_.translation() + (1-interpolation)*state1.t_wv_.translation();
                Eigen::Quaterniond rot = state1.t_wv_.so3().unit_quaternion().slerp(interpolation,state2.t_wv_.so3().unit_quaternion());
                stateOut.t_wv_ = Sophus::SE3d(rot,trans);
                break;
            }else{
                currTime = vStates[ii+1].timestamp_;
            }
        }
        return stateOut;
    }

    void UpdateWheels(const std::vector<Sophus::SE3d>& vWheelTransforms){
        Sophus::SE3d bodyT = t_wv_;
        wheel_omegas_.resize(4);
        wheel_states_.resize(4);
        for(size_t ii = 0 ; ii < wheel_omegas_.size() ; ii++){
            //position the wheels with respect to the body
            fflush(stdout);
            Sophus::SE3d T = bodyT* vWheelTransforms[ii];
            wheel_states_[ii] = T;
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
        VehicleState::AlignWithVelocityVector(t_wv_,vel_w_dot_);
    }

    /// Checks all ground contact points, and returns true if all wheels are off the ground
    bool IsAirborne() const
    {
        bool ground = false;
        if(wheel_omegas_.size() == 4 )
        {
            for(unsigned int ii = 0 ; ii < 4 ; ii++){
                if(wheel_omegas_[ii] == true) {
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
        forward = GetBasisVector(t_wv_,0); //get the forward
        return atan2(forward[1],forward[0]);
    }

    /// This function gets a vehicle state and returns a 5d pose vector which is parametrized as
    /// [x,y,t,k,v] where t is the 2d angle, k is the path curvature and v is the velocity
    static Eigen::Vector6d VehicleStateToPose(const VehicleState& state //< The given state to construct the pose from
                                              )
    {
        Eigen::Vector6d poseOut;
        poseOut << state.t_wv_.translation()[0],
                   state.t_wv_.translation()[1],
                   state.t_wv_.translation()[2],
                   state.GetTheta(),
                   state.curvature_,
                   state.vel_w_dot_.norm();
        return poseOut;
    }



    Sophus::SE3d t_wv_;                     //< 4x4 matrix denoting the state of the car
    std::vector<Sophus::SE3d> wheel_states_;   //< 4x4 matrices which denote the pose of each wheel
    std::vector<bool> wheel_omegas_;         //< Angular velocity of the vehicle in world coordinates

    Eigen::Vector3d vel_w_dot_;                       //< Linear velocity of the vehicle in world coordinates
    Eigen::Vector3d omega_w_dot_;                       //< Angular velocity of the vehicle in world coordinates

    double curvature_;                        //< The curvature at this point in the path
    double steering_cmd_;                         //< The steering command given to the car at this point (used to reset rate limitations)
    double timestamp_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
