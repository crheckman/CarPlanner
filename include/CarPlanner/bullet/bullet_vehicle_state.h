#pragma once

#include <CarPlanner/vehicle_state.h>
#include <CarPlanner/bullet/raycast_vehicle.h>

/// Class containing necessary description for BulletVehicle's state.
class BulletVehicleState : public VehicleState
{
public:

    BulletVehicleState() {}
    ~BulletVehicleState() {}

    void LoadState(RaycastVehicle *pVehicle)
    {
        //copy back the data
        *pVehicle = m_pVehicleBuffer;
        memcpy( (void*)pVehicle->getRigidBody(), m_pChassisBuffer,sizeof(RaycastVehicle));
    }

    void SaveState(RaycastVehicle *pVehicle)
    {
        //make a backup of the vhicle
        m_pVehicleBuffer = *pVehicle;
        memcpy(m_pChassisBuffer, (void*)pVehicle->getRigidBody(),sizeof(btRigidBody));
    }

private:
    unsigned char m_pChassisBuffer[sizeof(btRigidBody)];
    RaycastVehicle m_pVehicleBuffer;
};
