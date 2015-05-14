#pragma once

#include <CarPlanner/utils/vector.h>

/// Currently rely on "which world": GetSteeringRequiredAndMaxForce
/// Currently rely on bullet types: btTriangleMesh, btCollisionShape

namespace carplanner {

class VehicleWorld
{

public:

  virtual Eigen::Vector3d GetUpVector( int upAxis, double regularValue, double upValue ) = 0;
  virtual void Init(btCollisionShape *pCollisionShape,
                    const Eigen::Vector3d& dMin,
                    const Eigen::Vector3d& dMax,
                    std::map<int, double>& parameters,
                    unsigned int numWorlds ) = 0;
  virtual void DebugDrawWorld(int worldId) = 0;
  virtual void Init(const struct aiScene *pAIScene,
            std::map<int, double>& parameters,
            unsigned int numWorlds ) = 0;

  double GetTotalGravityForce(BulletWorldInstance* pWorld);



};

}
