#pragma once
#include <vector>
#include <sophus/se3.hpp>
#include <memory>

#include <CarPlanner/control/control_impl.h>
#include <CarPlanner/plan/planner_impl.h>
#include <CarPlanner/control_command.h>
#include <CarPlanner/ninjacar_impl.h>

namespace CarPlanner {
/*
 * CRTP for CarPlanner based on what kind of vehicle, planner and controller we
 * are using.
 *
 */

template<typename Vehicle,
         typename Planner,
         typename Controller>

class NinjaCar {
protected:
  typedef Eigen::Matrix<double,2,1> Vec2d;
  typedef Eigen::Matrix<double,3,1> Vec3d;
  typedef Sophus::SE3Group<double> SE3d;

public:
  NinjaCar() {}

  NinjaCar( const NinjaCar<Vehicle,Planner,Controller>& other ) :
     params_(other.params_), state_(other.state_) {}

  virtual ~NinjaCar() {}

  /* Now define the available methods for CarPlanner in pure virtual */

  /// Interpolate waypoint list.
  virtual std::vector<SE3d> GetPlan(const std::vector<SE3d>& waypoints) const = 0;

  /// Calculate instantaneous car control inputs in a vector.
  virtual CommandList GetControl( const Eigen::VectorXd& target_point) const = 0;

  /// Pose must be implemented by the CarModel.
  /// There is no SetPose as a result.
  virtual SE3d Pose() const = 0;

  /* Provide get/set methods for member variables we want to be public. */

  /// Get/Set for State.
  virtual void SetState( const Eigen::VectorXd& state ) {
    state_ = state;
  }

  virtual Eigen::VectorXd GetState() const {
    return state_;
  }

  /// Get/Set for Params.
  virtual void SetParams( const Eigen::VectorXd& params ) {
    params_ = params;
  }

  virtual Eigen::VectorXd GetParams() const {
    return params_;
  }

  /// Get/Set for Name.
  virtual void SetName( const std::string& name ) {
    name_ = name;
  }

  virtual std::string GetName() const {
    return name_;
  }

protected:
  NinjaCar( const Eigen::VectorXd& params_in,
            const Eigen::VectorXd& state_in )
    : params_(params_in), state_(state_in) {
  }

  Eigen::VectorXd params_;
  std::string name_;
  std::string type_;
  Eigen::VectorXd state_;
  std::vector<std::vector<double>> control_list_;

};

} //namespace carplanner
