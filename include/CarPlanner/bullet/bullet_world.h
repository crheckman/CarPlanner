#pragma once

#include <CarPlanner/world.h>

namespace carplanner {

class BulletWorld : public VehicleWorld {
  static btVector3 GetUpVector(int upAxis,btScalar regularValue,btScalar upValue);
  static void GenerateStaticHull(const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh , btVector3& dMin, btVector3& dMax);


};

}
