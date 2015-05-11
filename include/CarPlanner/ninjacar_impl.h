#pragma once
#include <CarPlanner/ninjacar.h>

namespace carplanner {

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
  NinjaCar<Vehicle>( params, state ) {}

  /// not defining any Vehicle-level methods here.
  /// if we choose to define other Vehicle types at some point, we can
  /// determine which methods are required to use MPC.

  };

} //namespace carplanner
