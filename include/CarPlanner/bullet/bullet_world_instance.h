#pragma once

#include <thread>
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <BulletDynamics/btBulletDynamicsCommon.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>

#include <CarPlanner/bullet/gl_debug_drawer.h>
#include <CarPlanner/bullet/raycast_vehicle.h>
#include <CarPlanner/vehicle_state.h>

struct BulletWorldInstance : public std::mutex
{
    BulletWorldInstance()
    {
        m_pCarChassis = NULL;
        m_pVehicle = NULL;
        m_pTerrainShape = NULL;
        m_pHeightfieldData = NULL;
        timestamp_ = -1;
        m_bParametersChanged = false;
        total_command_time_ = 0;
    }

    ~BulletWorldInstance()
    {
        if( m_pTerrainShape != NULL) {
            delete m_pTerrainShape;
        }
    }

    std::vector<Sophus::SE3d> m_vWheelTransforms;
    double timestamp_;

    btScalar *m_pHeightfieldData;
    btCollisionShape *m_pTerrainShape;
    RaycastVehicle::btVehicleTuning	m_Tuning;
    btVehicleRaycaster*	m_pVehicleRayCaster;
    RaycastVehicle*	m_pVehicle;
    btCollisionShape* m_pVehicleChassisShape;

    btAlignedObjectArray<btCollisionShape*> m_vCollisionShapes;
    btAlignedObjectArray<btCollisionShape*> m_vVehicleCollisionShapes;
    class btDefaultCollisionConfiguration* m_pCollisionConfiguration;
    class btCollisionDispatcher*	m_pDispatcher;

    class btBroadphaseInterface*	m_pOverlappingPairCache;
    class btConstraintSolver*	m_pConstraintSolver;
    class btDiscreteDynamicsWorld* m_pDynamicsWorld;

    btRigidBody* m_pCarChassis;
    GLDebugDrawer m_DebugDrawer;
    VehicleState state_backup_;
    VehicleState state_;

    CommandList previous_commands_;    //< List holding the previous commands sent to the model (Newest commands at the front, oldest at the back)
    double total_command_time_;

    //static parameters
    std::map<int, double> m_Parameters;

    //VehicleParameters m_Parameters;
    //std::vector<RegressionParameter> m_vLearningParameters;

    int m_nIndex;
    bool m_bParametersChanged;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
