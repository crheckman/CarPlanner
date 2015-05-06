#pragma once
#include <memory>

#include <CarPlanner/utils/vector.h>
#include <CarPlanner/control_command.h>
#include <CarPlanner/ninjacar_impl.h>
#include <CarPlanner/vehicle_parameters.h>
#include <CarPlanner/vehicle_state.h>

namespace carplanner {
/*
 * Polymorphism for CarPlanner based on what kind of vehicle, planner and
 * controller we are using.
 *
 */

template<typename Vehicle,
         typename Controller>

class NinjaCar {
protected:
  typedef Eigen::Matrix<double,2,1> Vec2d;
  typedef Eigen::Matrix<double,3,1> Vec3d;

public:
  NinjaCar() {}

  NinjaCar( const NinjaCar<Vehicle,Controller>& other ) :
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
  virtual void SetState( const VehicleState& state ) {
    state_ = state;
  }

  virtual VehicleState GetState() const {
    return state_;
  }

  /// Get for Vehicle and Controller.
  /// Will simply return a string based on which we're using.
  virtual std::string GetVehicleModel() = 0;

  virtual std::string GetController() = 0;

  /// Get/Set for Params.
  virtual void SetParams( const VehicleParameters& params ) {
    params_ = params;
  }

  virtual VehicleParameters GetParams() const {
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
  NinjaCar( const VehicleParameters& params_in,
            const VehicleState& state_in )
    : params_(params_in), state_(state_in) {
  }

  std::string name_;
  std::string type_;
  VehicleParameters params_;
  VehicleState state_;
  std::vector<std::vector<double>> control_list_;

};

} //namespace carplanner
