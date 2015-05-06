#pragma once
#include <CarPlanner/control/control_methods.h>
#include <CarPlanner/plan/planner_methods.h>
#include <CarPlanner/ninjacar.h>

namespace CarPlanner {

/// Using a Bullet car model.
template <typename Vehicle = BulletCarModel,
          typename Planner,
          typename Controller,
          typename Derived>

class BulletNinja : public NinjaCar<Vehicle, Planner, Controller> {
    typedef typename NinjaCar<Vehicle, Planner, Controller>::Vec2d Vec2d;
    typedef typename NinjaCar<Vehicle, Planner, Controller>::Vec3d Vec3d;
    typedef typename NinjaCar<Vehicle, Planner, Controller>::SE3d SE3d;

public:

  BulletNinja() {}
  virtual ~BulletNinja() {}
  BulletNinja( const Eigen::VectorXd& params,
               const Eigen::VectorXd& state ) :
    NinjaCar<Vehicle, Planner, Controller>( params, state ) {
  }

  std::vector<SE3d>
  Plan( const std::vector<SE3d>& waypoints ) override {
    Derived::Plan(start_point, end_point, this);
  }

  std::vector<double>
  GetControl( const SE3d& target_point ) override {
    Derived::Control(target_point, this->state_.data(), this->params_.data());
  }

  SE3d
  Pose() override {
    Sophus::SE3d poseOut;
    poseOut << state_.m_dTwv.translation()[0],
               state_.m_dTwv.translation()[1],
               state_.m_dTwv.translation()[2],
               state_.GetTheta(),
               state_.m_dCurvature,
               state_.m_dV.norm();
    std::cout << "Check proper formatting of state to Pose for Bullet car." << std::endl; //crh unfinished
    return poseOut;
  }

  };

} //namespace CarPlanner
