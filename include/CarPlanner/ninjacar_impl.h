#pragma once
#include <CarPlanner/control/control_methods.h>
#include <CarPlanner/ninjacar.h>

namespace carplanner {

/// Using a Bullet car model.
template <typename Vehicle, typename Derived>

class NinjaImpl : public NinjaCar<Vehicle> {
    typedef typename NinjaCar<Vehicle>::Vec2d Vec2d;
    typedef typename NinjaCar<Vehicle>::Vec3d Vec3d;
    typedef typename NinjaCar<Vehicle>::SE3d SE3d;

public:

  NinjaImpl() {}
  virtual ~NinjaImpl() {}
  NinjaImpl( const Eigen::VectorXd& params,
               const Eigen::VectorXd& state ) :
    NinjaCar<Vehicle>( params, state ) {
  }

  SE3d
  Pose() override {
    Sophus::SE3d pose_out_;
    pose_out_ << state_.t_wv_.translation()[0],
               state_.t_wv_.translation()[1],
               state_.t_wv_.translation()[2],
               state_.GetTheta(),
               state_.curvature_,
               state_.vel_w_dot_.norm();
    std::cout << "Check proper formatting of state to Pose for Bullet car." << std::endl; //crh unfinished
    return pose_out_;
  }

  };

} //namespace carplanner
