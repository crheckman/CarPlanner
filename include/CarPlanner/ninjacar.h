#pragma once
#include <memory>
#include <glog/logging.h>

#include <assimp/scene.h>
#include <CarPlanner/utils/vector.h>
#include <CarPlanner/control_command.h>
#include <CarPlanner/vehicle_state.h>

namespace carplanner {
/*
 * Polymorphism for CarPlanner based on what kind of vehicle, planner and
 * controller we are using.
 *
 */

/// Structure containing all parameters for a car
struct VehicleParameters
{
  static const char * const Names[];
  static bool LoadFromFile(const std::string sFile, std::map<int, double> &map);
  static void PrintAllParams(const std::map<int, double> &map);
  static bool SaveToFile(const std::string sFile, const std::map<int, double> &map);
};

class NinjaCar {
protected:
  typedef Eigen::Matrix<double,2,1> Vec2d;
  typedef Eigen::Matrix<double,3,1> Vec3d;
  typedef Sophus::SE3d SE3d;

public:
  NinjaCar() {}
  NinjaCar(const NinjaCar& other) :
    name_(other.name_), type_(other.type_),
    state_(other.state_), params_(other.params_) {}

  virtual ~NinjaCar() {}

  /* Now define the available methods for CarPlanner in pure virtual */

  /// Pose must be implemented by the CarModel.
  /// There is no SetPose as a result.
  //virtual SE3d Pose() const = 0;

  /* Provide get/set methods for member variables we want to be public. */

  /// Get/Set for State.
  virtual void SetVehicleState( const int world_id, const VehicleState& state ) = 0;

  virtual VehicleState GetVehicleState( const int world_id, const VehicleState& state_out ) const = 0;

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

  virtual void UpdateParameters( VehicleParameters& params ) = 0;

  virtual void GenerateStaticHull(const struct aiScene *pAIScene,
                                  const struct aiNode *pAINode,
                                  const aiMatrix4x4 parentTransform,
                                  const float flScale,
                                  btTriangleMesh &triangleMesh, //crh
                                  Eigen::Vector3d& dMin,
                                  Eigen::Vector3d& dMax) = 0;

  std::pair<double, double> GetSteeringRequiredAndMaxForce(const int world_id,
                                                           const int nWheelId,
                                                           const double dPhi,
                                                           const double dt);
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
  VehicleParameters params_;

};

} //namespace carplanner
