#pragma once
#include <memory>

#include <CarPlanner/utils/vector.h>
#include <CarPlanner/control_command.h>
#include <CarPlanner/vehicle_state.h>

namespace carplanner {
/*
 * Polymorphism for CarPlanner based on what kind of vehicle, planner and
 * controller we are using.
 *
 */

class NinjaCar {
protected:
  typedef Eigen::Matrix<double,2,1> Vec2d;
  typedef Eigen::Matrix<double,3,1> Vec3d;
  typedef Sophus::SE3d SE3d;

public:
  NinjaCar() {}
  virtual ~NinjaCar() {}

  /* Now define the available methods for CarPlanner in pure virtual */

  /// Pose must be implemented by the CarModel.
  /// There is no SetPose as a result.
  //virtual SE3d Pose() const = 0;

  /* Provide get/set methods for member variables we want to be public. */

  /// Get/Set for State.
  virtual void SetState( const VehicleState& state ) {
    state_ = state;
  }

  virtual VehicleState GetState() const {
    return state_;
  }

  /// Get for Vehicle.
  /// Will simply return a string based on which we're using.
  //virtual std::string VehicleModel() = 0;

  /// Get/Set for Name.
  virtual void SetName( const std::string& name ) {
    name_ = name;
  }

  virtual std::string GetName() const {
    return name_;
  }

protected:
  /// This constructor isn't needed yet.
/*  NinjaCar( const VehicleParameters& params_in,
            const VehicleState& state_in )
    : params_(params_in), state_(state_in) {
  }
*/

  std::string name_;
  std::string type_;
  VehicleState state_;

};

} //namespace carplanner
