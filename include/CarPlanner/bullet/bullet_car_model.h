/*
 * File:   bullet_car_model.h
 * Author: nima
 * 		   crh
 *
 */

#pragma once
#include <assimp/scene.h>

#include <sophus/se3.hpp>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>

#include <CarPlanner/bullet/bullet_car_parameters.h>
#include <CarPlanner/bullet/bullet_world_instance.h>

#define CAR_UP_AXIS 2   //this is the index for the bullet Z axis
#define CAR_FORWARD_AXIS 0   //this is the index for the bullet X axis
#define CAR_RIGHT_AXIS 1   //this is the index for the bullet Y axis
#define CUBE_HALF_EXTENTS 1

#define BULLET_MODEL_GRAVITY 9.81

#define MAX_SERVO_ANGLE 45


//collision filtering
#define COL_NOTHING 0
#define COL_RAY 1   //this is the "default" filter that we need to include for raycasting (on the ground only)
#define COL_CAR 2
#define COL_GROUND 4

//vehicle parameter ordering
#define VEHICLE_NUM_PARAMS 5
#define VEHICLE_WIDTH 0.21
#define VEHICLE_WHEEL_BASE 0.27
#define VEHICLE_WHEEL_RADIUS 0.04
#define VEHICLE_WHEEL_WIDTH 0.025
#define MIN_CONTROL_DELAY 0.0
#define MAX_CONTROL_DELAY 0.3





class BulletCarModel
{
public:
    BulletCarModel();
    ~BulletCarModel();

    static btVector3 GetUpVector(int upAxis,btScalar regularValue,btScalar upValue);
    /////////////////////////////////////////////////////////////////////////////////////////
    static void GenerateStaticHull(const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh , btVector3& dMin, btVector3& dMax);
    void Init(btCollisionShape *pCollisionShape, const btVector3 &dMin, const btVector3 &dMax, std::map<int, double> &parameters, unsigned int numWorlds );
    void Init(const struct aiScene *pAIScene,std::map<int, double>& parameters, unsigned int numWorlds );
    void DebugDrawWorld(int worldId);

    std::pair<double, double> GetSteeringRequiredAndMaxForce(const int world_id, const int nWheelId, const double dPhi, const double dt);
    double GetTotalGravityForce(BulletWorldInstance* pWorld);
    double GetTotalWheelFriction(int worldId, double dt);
    double _CalculateWheelFriction(int wheelNum, BulletWorldInstance* pInstance, double dt);
    /////////////////////////////////////////////////////////////////////////////////////////
    void UpdateState(const int &worldId,
                     const ControlCommand command,
                     const double forceDt = -1,
                     const bool bNoDelay = false,
                     const bool bNoUpdate  = false );
    /////////////////////////////////////////////////////////////////////////////////////////
    virtual void VehicleState(int worldId, VehicleState &stateOut);
    //virtual VehicleState VehicleStateAsync(int worldId);
    Eigen::Vector3d VehicleLinearVelocity(int worldId);
    Eigen::Vector3d VehicleAngularVelocity(int worldId);
    Eigen::Vector3d VehicleInertiaTensor(int worldId);
    virtual void SetState(int world_id,  const VehicleState& state );
    //virtual void SetState( int worldId,  const Eigen::Matrix4d& vState  );
    virtual void SetStateNoReset(BulletWorldInstance *pWorld, const Sophus::SE3d &Twv );
    BulletWorldInstance *GetWorldInstance(int id){ return m_vWorlds[id]; }
    const BulletWorldInstance *GetWorldInstance(int id) const { return m_vWorlds[id]; }
    const int GetWorldCount() const { return m_vWorlds.size(); }

    double GetCorrectedSteering(double& dCurvature, int index);
    double  GetSteeringAngle(const double dcurvature, double& dCorrectedCurvature, int index, double steeringCoef /*= 1*/);
    std::vector<Sophus::SE3d> GetWheelTransforms(const int worldIndex);

    //getter functions
    Eigen::Vector3d     GetGravity() { return m_dGravity; }
    //HeightMap*          GetHeightMap() { return m_pHeightMap; }
    void                GetCommandHistory(int worldId,CommandList &previousCommandsOut);
    void                ResetCommandHistory(int worldId);
    CommandList&        GetCommandHistoryRef(int worldId);
    void                PushDelayedControl(int worldId, ControlCommand& delayedCommands);
    std::map<int, double>& GetParameters(int index) { return GetWorldInstance(index)->m_Parameters; }
    const std::map<int, double>& GetParameters(int index) const  { return GetWorldInstance(index)->m_Parameters; }
    void                SetCommandHistory(const int &worldId, const CommandList &previousCommands);
    bool RayCast(const Eigen::Vector3d& dSource, const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool &biDirectional, int index = 0);

    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams);
    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams, BulletWorldInstance *pWorld);
    void UpdateParameters(const std::vector<RegressionParameter>& vNewParams,int index);
    void _InternalUpdateParameters(BulletWorldInstance* pWorld);


protected:

    void _GetDelayedControl(int worldId, double timeDelay, ControlCommand& delayedCommands);
    btRigidBody*	_LocalCreateRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask);
    void _InitVehicle(BulletWorldInstance* pWorld, std::map<int, double>& parameters);
    void _InitWorld(BulletWorldInstance* pWorld, btCollisionShape *pGroundShape, btVector3 dMin, btVector3 dMax, bool centerMesh);

    std::vector< BulletWorldInstance * > m_vWorlds;
    //HeightMap *m_pHeightMap;

    Eigen::Vector3d m_dGravity;
    unsigned int m_nNumWorlds;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

}; // class BulletCarModel
