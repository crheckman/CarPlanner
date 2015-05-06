#pragma once
#include <CarPlanner/control/control_methods.h>
#include <CarPlanner/plan/planner_methods.h>
#include <CarPlanner/ninjacar.h>

namespace carplanner {

/// Using a Bullet car model.
template <typename Vehicle = BulletCarModel,
          typename Planner,
          typename Controller,
          typename Derived>

class BulletNinja : public NinjaCar<Vehicle, Controller> {
    typedef typename NinjaCar<Vehicle, Controller>::Vec2d Vec2d;
    typedef typename NinjaCar<Vehicle, Controller>::Vec3d Vec3d;
    typedef typename NinjaCar<Vehicle, Controller>::SE3d SE3d;

public:

  BulletNinja() {}
  virtual ~BulletNinja() {}
  BulletNinja( const Eigen::VectorXd& params,
               const Eigen::VectorXd& state ) :
    NinjaCar<Vehicle, Controller>( params, state ) {
  }

  std::vector<SE3d>
  GetPlan( const std::vector<SE3d>& waypoints ) override {
    Derived::Plan(start_point, end_point, this);
  }

  std::vector<double>
  GetControl( const SE3d& target_point ) override {
    Derived::Control(target_point, this->state_.data(), this->params_.data());
  }

  SE3d
  Pose() override {
    Sophus::SE3d poseOut;
    poseOut << state_.t_wv_.translation()[0],
               state_.t_wv_.translation()[1],
               state_.t_wv_.translation()[2],
               state_.GetTheta(),
               state_.curvature_,
               state_.vel_w_dot_.norm();
    std::cout << "Check proper formatting of state to Pose for Bullet car." << std::endl; //crh unfinished
    return poseOut;
  }

  };

} //namespace carplanner