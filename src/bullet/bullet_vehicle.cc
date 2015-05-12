#include <assimp/cimport.h>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/material.h>

#include <CVars/CVar.h>

#include <CarPlanner/ninjacar.h>
#include <CarPlanner/utils/optim.h>
#include <CarPlanner/utils/vector.h>
#include <CarPlanner/bullet/bullet_world_instance.h>

typedef carplanner::BulletCarModel BulletCarModel;

BulletCarModel::BulletCarModel()
{
    m_dGravity << 0,0,BULLET_MODEL_GRAVITY;

}

/////////////////////////////////////////////////////////////////////////////////////////
BulletCarModel::~BulletCarModel()
{

}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::DebugDrawWorld(int worldId)
{
    BulletWorldInstance * pWorld = GetWorldInstance(worldId);
    if( pWorld->m_pDynamicsWorld != NULL ) {
        pWorld->m_pDynamicsWorld->debugDrawWorld();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
btVector3 BulletCarModel::GetUpVector(int upAxis,btScalar regularValue,btScalar upValue)
{
    btAssert(upAxis >= 0 && upAxis <= 2 && "bad up axis");

    btVector3 v(regularValue, regularValue, regularValue);
    v[upAxis] = upValue;

    return v;
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::GenerateStaticHull( const struct aiScene *pAIScene, const struct aiNode *pAINode, const aiMatrix4x4 parentTransform, const float flScale, btTriangleMesh &triangleMesh, btVector3& dMin, btVector3& dMax )
{

    aiMesh *pAIMesh;

    aiFace *pAIFace;

    for (size_t x = 0; x < pAINode->mNumMeshes; x++ )
    {
        pAIMesh = pAIScene->mMeshes[pAINode->mMeshes[x]];

        for (size_t y = 0; y < pAIMesh->mNumFaces; y++ )
        {
            pAIFace = &pAIMesh->mFaces[y];

            if ( pAIFace->mNumIndices != 3 )
            {
                /*if ( bEnableDebugging )
                {
                    std::cout << "WARNING: A non-triangle face has been detected on this mesh, which is currently not supported." << std::endl;
                    std::cout << "         As such, this face will not be used to generate a collision shape for this mesh." << std::endl;
                    std::cout << "         This could have disastrous consequences. Consider using a different mesh!" << std::endl;
                }*/
                continue;
            }


            aiVector3D v1 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[0]];
            aiVector3D v2 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[1]];
            aiVector3D v3 = parentTransform * pAIMesh->mVertices[pAIFace->mIndices[2]];

            dMin[0] = std::min((float)dMin[0],std::min(v1.x,std::min(v2.x, v3.x)));
            dMax[0] = std::max((float)dMax[0],std::min(v1.x,std::max(v2.x, v3.x)));

            dMin[1] = std::min((float)dMin[1],std::min(v1.y,std::min(v2.y, v3.y)));
            dMax[1] = std::max((float)dMax[1],std::max(v1.y,std::max(v2.y, v3.y)));

            dMin[2] = std::min((float)dMin[2],std::min(v1.z,std::min(v2.z, v3.z)));
            dMax[2] = std::max((float)dMax[2],std::max(v1.z,std::max(v2.z, v3.z)));

            triangleMesh.addTriangle( btVector3(v1.x * flScale, v1.y * flScale, v1.z * flScale),
                                      btVector3(v2.x * flScale, v2.y * flScale, v2.z * flScale),
                                      btVector3(v3.x * flScale, v3.y * flScale, v3.z * flScale),
                                      false );
        }
    }

    for (size_t x = 0; x < pAINode->mNumChildren; x++ ){
        GenerateStaticHull( pAIScene, pAINode->mChildren[x],parentTransform*pAINode->mChildren[x]->mTransformation, flScale, triangleMesh,dMin,dMax );
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::Init(btCollisionShape* pCollisionShape, const btVector3 &dMin, const btVector3 &dMax, std::map<int, double> &parameters, unsigned int numWorlds )
{
    m_nNumWorlds = numWorlds;
    //initialize a number of worlds
    for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
        BulletWorldInstance *pWorld = new BulletWorldInstance();
        pWorld->m_nIndex = ii;

        _InitWorld(pWorld,pCollisionShape,dMin,dMax,false);

        //initialize the car
        _InitVehicle(pWorld,parameters);

        m_vWorlds.push_back(pWorld);
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::Init(const struct aiScene *pAIScene, std::map<int, double> &parameters, unsigned int numWorlds )
{
    aiNode *pAINode = pAIScene->mRootNode;

    //generate the triangle mesh
    btVector3 dMin(DBL_MAX,DBL_MAX,DBL_MAX);
    btVector3 dMax(DBL_MIN,DBL_MIN,DBL_MIN);
    btTriangleMesh *pTriangleMesh = new btTriangleMesh();
    GenerateStaticHull(pAIScene,pAINode,pAINode->mTransformation,1.0,*pTriangleMesh,dMin,dMax);

    m_nNumWorlds = numWorlds;
    //initialize a number of worlds
    for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
        BulletWorldInstance *pWorld = new BulletWorldInstance();
        pWorld->m_nIndex = ii;

        btCollisionShape* pCollisionShape = new btBvhTriangleMeshShape(pTriangleMesh,true,true);

        //initialize the world (heightmap and general physics)
        _InitWorld(pWorld,pCollisionShape,dMin,dMax,false);

        //initialize the car
        _InitVehicle(pWorld,parameters);

        m_vWorlds.push_back(pWorld);
    }

}


//////////////////////////////////////////////s///////////////////////////////////////////
void BulletCarModel::PushDelayedControl(int worldId, ControlCommand& delayedCommands)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    //lock the world to prevent changes
    std::lock_guard<std::mutex> lock(*pWorld);
    pWorld->previous_commands_.push_front(delayedCommands);
    //add to the total command time
    pWorld->total_command_time_ += delayedCommands.timestep_;
    //int s = pWorld->previous_commands_.size();

    //if this time is over the maximum amount, pop them off at the back
    while(pWorld->total_command_time_ > MAX_CONTROL_DELAY &&
              (pWorld->total_command_time_ - pWorld->previous_commands_.back().timestep_) > MAX_CONTROL_DELAY)
    {
        pWorld->total_command_time_ -= pWorld->previous_commands_.back().timestep_;
        pWorld->previous_commands_.pop_back();

    }

}

/////////////////////////////////////////////////////////////////////////////////////////
//std::vector<RegressionParameter> & BulletCarModel::GetLearningParameterVector(int index)
//{
//    BulletWorldInstance* pWorld = GetWorldInstance(index);
//    //update the parameter vector
//    for(int ii = 0 ; ii < pWorld->m_vLearningParameters.size() ; ii++) {
//        pWorld->m_vLearningParameters[ii].val_ = pWorld->m_Parameters[pWorld->m_vLearningParameters[ii].key_];
//    }
//    return pWorld->m_vLearningParameters;
//}

///////////////////////////////////////////////////////////////////////////////////////////
//void SetLearningParameterVector(const std::vector<RegressionParameter> &params)
//{
//    for(int ii = 0 ; ii < m_nNumWorlds ; ii++) {
//        SetLearningParameterVector(params,ii);
//    }
//}

///////////////////////////////////////////////////////////////////////////////////////////
//void SetLearningParameterVector(const std::vector<RegressionParameter> &params,const int& index)
//{
//    BulletWorldInstance* pWorld = GetWorldInstance(index);
//    pWorld->m_vLearningParameters = params;
//}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_GetDelayedControl(int worldId, double timeDelay, ControlCommand& delayedCommands)
{
    //clamp the time delay to be > 0
    timeDelay = std::max(timeDelay,0.0);

    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    //lock the world to prevent changes
    std::lock_guard<std::mutex> lock(*pWorld);
    CommandList& previousCommands = pWorld->previous_commands_;
    //if there is control delay, get commands from a previous time
    double currentDelay = 0;
    CommandList::iterator it = previousCommands.begin();
    ControlCommand* pCurrentCommand = &(*it);
    //currentDelay = (*it).timestep_;


    int count = 0;
    if(previousCommands.size() > 1) {
        it++; //move to the first element
        for(; it != previousCommands.end() ; it++) {
            count++;
            if( currentDelay + (*it).timestep_ >= timeDelay ) {

                //interpolate between the current and next commands
                double r2 = (timeDelay - currentDelay)/(*it).timestep_;
                double r1 = 1-r2;

                delayedCommands.force_ = r1*pCurrentCommand->force_ + r2*(*it).force_;
                delayedCommands.curvature_ = r1*pCurrentCommand->curvature_ + r2*(*it).curvature_;
                delayedCommands.phi_ = r1*pCurrentCommand->phi_ + r2*(*it).phi_;
                delayedCommands.torque_ = r1*pCurrentCommand->torque_ + r2*(*it).torque_;

                it++;
                return;
            }else {
                pCurrentCommand = &(*it);
                currentDelay += pCurrentCommand->timestep_;

                if(currentDelay == timeDelay) {
                    delayedCommands = *pCurrentCommand;
                    return;
                }
            }
        }
    }else if(previousCommands.size() > 0){
        dout("Command history list size < 2, using first command.");
        delayedCommands = previousCommands.front();
    }else{
        dout("Command history list size == 0. Passing empty command");
        delayedCommands.force_ = pWorld->m_Parameters[BulletVehicleParameters::AccelOffset]*SERVO_RANGE;
        delayedCommands.curvature_ = 0;
        delayedCommands.phi_ = pWorld->m_Parameters[BulletVehicleParameters::SteeringOffset]*SERVO_RANGE;
        delayedCommands.torque_ << 0,0,0;
    }


}

/////////////////////////////////////////////////////////////////////////////////////////
double BulletCarModel::GetCorrectedSteering(double& dCurvature, int index)
{
    BulletWorldInstance* pWorld = GetWorldInstance(index);
    double phi = atan(dCurvature*pWorld->m_Parameters[BulletVehicleParameters::WheelBase]);
    phi = SoftMinimum(pWorld->m_Parameters[BulletVehicleParameters::MaxSteering],
          SoftMaximum(phi,-pWorld->m_Parameters[BulletVehicleParameters::MaxSteering],50),50);
    dCurvature  = (tan(phi)/pWorld->m_Parameters[BulletVehicleParameters::WheelBase]);
    return phi;
}

/////////////////////////////////////////////////////////////////////////////////////////
double BulletCarModel::GetSteeringAngle(const double dcurvature, double& dCorrectedCurvature, int index, double steeringCoef /*= 1*/)
{
    BulletWorldInstance* pWorld = GetWorldInstance(index);
    double phi = atan(dcurvature*pWorld->m_Parameters[BulletVehicleParameters::WheelBase])*steeringCoef;
    //double phi = 1.0/atan((1-powi(pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2,2)*powi(dcurvature,2))/(powi(dcurvature,2)*powi(pWorld->m_Parameters[BulletVehicleParameters::WheelBase],2)));

    dCorrectedCurvature  = (tan(phi)/pWorld->m_Parameters[BulletVehicleParameters::WheelBase]);
    //dCorrectedCurvature = 1/sqrt(powi(pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2,2) + powi(pWorld->m_Parameters[BulletVehicleParameters::WheelBase],2)*powi(1.0/tan(phi),2));
    return phi;
}


/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::UpdateParameters(const std::vector<RegressionParameter>& vNewParams)
{
    for(size_t ii = 0 ; ii < m_nNumWorlds ; ii++) {
        UpdateParameters(vNewParams,ii);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::UpdateParameters(const std::vector<RegressionParameter>& vNewParams,int index) {
    UpdateParameters(vNewParams,GetWorldInstance(index));
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::UpdateParameters(const std::vector<RegressionParameter>& vNewParams,BulletWorldInstance* pWorld)
{
    std::lock_guard<std::mutex> lock(*pWorld);
    //update the parameter map and learning parameter list with the new params
    for(size_t ii = 0; ii < vNewParams.size() ; ii++) {
        //pWorld->m_vLearningParameters[ii].val_ = vNewParams[ii].val_;
        pWorld->m_Parameters[vNewParams[ii].key_] = vNewParams[ii].val_;
        //dout("Updating parameter with key " << vNewParams[ii].key_ << " to " << pWorld->m_Parameters[vNewParams[ii].key_]);
    }
    _InternalUpdateParameters(pWorld);
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_InternalUpdateParameters(BulletWorldInstance* pWorld)
{
    //dout("updating parameters to " << pWorld->m_vParameters);

    pWorld->m_bParametersChanged = true;

    //dyanmic friction is slightly less
    pWorld->vehicle_->SetDynamicFrictionCoefficient(pWorld->m_Parameters[BulletVehicleParameters::DynamicFrictionCoef]);
    //set side friction (for drifting)
    pWorld->vehicle_->SetStaticSideFrictionCoefficient(pWorld->m_Parameters[BulletVehicleParameters::StaticSideFrictionCoef]);
    pWorld->vehicle_->SetSlipCoefficient(pWorld->m_Parameters[BulletVehicleParameters::SlipCoefficient]);
    pWorld->vehicle_->SetMagicFormulaCoefficients(pWorld->m_Parameters[BulletVehicleParameters::MagicFormula_B],
                                                    pWorld->m_Parameters[BulletVehicleParameters::MagicFormula_C],
                                                    pWorld->m_Parameters[BulletVehicleParameters::MagicFormula_E]);

    //set the mass and wheelbase of the car
    //pWorld->m_Parameters[BulletVehicleParameters::WheelBase] = pWorld->m_vParameters[eWheelBase];
    //pWorld->m_Parameters.m_dMass = pWorld->m_vParameters[eMass];
    pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->vehicle_->getRigidBody());
    //change rigid body dimensions
    btBoxShape *pBoxShape =  (btBoxShape *)pWorld->vehicle_->getRigidBody()->getCollisionShape();
    pBoxShape->setImplicitShapeDimensions(btVector3(pWorld->m_Parameters[BulletVehicleParameters::WheelBase],pWorld->m_Parameters[BulletVehicleParameters::Width],pWorld->m_Parameters[BulletVehicleParameters::Height]));
    //calculate new inertia
    btVector3 localInertia(0,0,0);
    pBoxShape->calculateLocalInertia(pWorld->m_Parameters[BulletVehicleParameters::Mass],localInertia);
    pWorld->vehicle_->getRigidBody()->setMassProps(pWorld->m_Parameters[BulletVehicleParameters::Mass],localInertia);
    pWorld->m_pDynamicsWorld->addRigidBody(pWorld->vehicle_->getRigidBody(),COL_CAR,COL_NOTHING);

    //change the position of the wheels
    pWorld->vehicle_->getWheelInfo(0).m_chassisConnectionPointCS[0] = pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2;
    pWorld->vehicle_->getWheelInfo(1).m_chassisConnectionPointCS[0] = pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2;
    pWorld->vehicle_->getWheelInfo(2).m_chassisConnectionPointCS[0] = -pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2;
    pWorld->vehicle_->getWheelInfo(3).m_chassisConnectionPointCS[0] = -pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2;

    for (size_t i=0;i<pWorld->vehicle_->getNumWheels();i++)
    {
        WheelInfo& wheel = pWorld->vehicle_->getWheelInfo(i);
        pWorld->vehicle_->updateWheelTransformsWS(wheel);
        pWorld->vehicle_->updateWheelTransform(i);
        wheel.m_suspensionRestLength1 = pWorld->m_Parameters[BulletVehicleParameters::SuspRestLength];
        wheel.m_suspensionStiffness = pWorld->m_Parameters[BulletVehicleParameters::Stiffness];
        wheel.m_wheelsDampingCompression = pWorld->m_Parameters[BulletVehicleParameters::CompDamping];
        wheel.m_wheelsDampingRelaxation = pWorld->m_Parameters[BulletVehicleParameters::ExpDamping];
    }

    pWorld->vehicle_->updateSuspension();

}

/////////////////////////////////////////////////////////////////////////////////////////
double BulletCarModel::GetTotalWheelFriction(int worldId, double dt)
{
    double totalForce = 0;
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    for(size_t ii = 0; ii < pWorld->vehicle_->getNumWheels() ; ii++) {
        totalForce += _CalculateWheelFriction((ii),pWorld,dt);
    }
    return totalForce;
}

/////////////////////////////////////////////////////////////////////////////////////////
std::pair<double,double> BulletCarModel::GetSteeringRequiredAndMaxForce(const int world_id, const int nWheelId, const double dPhi, const double dt)
{
    BulletWorldInstance* pWorld = GetWorldInstance(world_id);
    return pWorld->vehicle_->GetSteeringRequiredAndMaxForce(nWheelId,dPhi,dt);
}

/////////////////////////////////////////////////////////////////////////////////////////
double BulletCarModel::_CalculateWheelFriction(int wheelNum, BulletWorldInstance* pInstance, double dt)
{
    bool bDynamic;
    double maxImpulse = pInstance->vehicle_->CalculateMaxFrictionImpulse(wheelNum,dt,bDynamic);
    return maxImpulse / dt;
}

/////////////////////////////////////////////////////////////////////////////////////////
double BulletCarModel::GetTotalGravityForce(BulletWorldInstance* pWorld)
{
    btVector3 gravityForce(0,0,-10*pWorld->m_Parameters[BulletVehicleParameters::Mass]);
    for(size_t ii = 0; ii < pWorld->vehicle_->getNumWheels() ; ii++) {
        WheelInfo& wheel = pWorld->vehicle_->getWheelInfo(ii);
        if(wheel.m_raycastInfo.m_isInContact)
        {
            gravityForce -= wheel.m_raycastInfo.m_contactNormalWS*wheel.m_wheelsSuspensionForce;
        }
    }
    //now get the component in the direction of the vehicle
    const btTransform& chassisTrans = pWorld->vehicle_->getChassisWorldTransform();
    btVector3 carFwd (
                chassisTrans.getBasis()[0][CAR_FORWARD_AXIS],
                chassisTrans.getBasis()[1][CAR_FORWARD_AXIS],
                chassisTrans.getBasis()[2][CAR_FORWARD_AXIS]);
    double force = gravityForce.dot(carFwd);
    return -force;
}



/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::UpdateState(  const int& worldId,
                                   const ControlCommand command,
                                   const double forceDt /*= -1*/,
                                   const bool bNoDelay /* = false */,
                                   const bool bNoUpdate /* = false */)
{
    ControlCommand delayedCommands;
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);

    if(pWorld->timestamp_ == -1 && forceDt == -1 )   {
        pWorld->timestamp_ = Tic();
        return;
    }

    //calculate the time since the last iteration
    double dT = Toc( pWorld->timestamp_ );
    pWorld->timestamp_ = Tic();

    if( forceDt != -1 ){
        dT = forceDt;
    }

    delayedCommands = command;
    delayedCommands.timestep_ = dT;
    if(bNoDelay == false){
        PushDelayedControl(worldId,delayedCommands);
        //get the delayed command
        _GetDelayedControl(worldId, pWorld->m_Parameters[BulletVehicleParameters::ControlDelay],delayedCommands);
    }
    //get the delayed commands for execution and remove the offsets
    double dCorrectedForce = delayedCommands.force_- pWorld->m_Parameters[BulletVehicleParameters::AccelOffset]*SERVO_RANGE;
    double dCorrectedPhi = delayedCommands.phi_-pWorld->m_Parameters[BulletVehicleParameters::SteeringOffset]*SERVO_RANGE;

    //D.C. motor equations:
    //torque = Pwm*Ts - slope*V
    //TODO: make this velocity in the direction of travel
    const double stallTorque = dCorrectedForce*pWorld->m_Parameters[BulletVehicleParameters::StallTorqueCoef];
    dCorrectedForce = sgn(stallTorque)*std::max(0.0,fabs(stallTorque) -
                      pWorld->m_Parameters[BulletVehicleParameters::TorqueSpeedSlope]*fabs(pWorld->state_.vel_w_dot_.norm()));

    //now apply the offset and scale values to the force and steering commands
    dCorrectedPhi = dCorrectedPhi/(pWorld->m_Parameters[BulletVehicleParameters::SteeringCoef]*SERVO_RANGE);

    //clamp the steering
    dCorrectedPhi = SoftMinimum(pWorld->m_Parameters[BulletVehicleParameters::MaxSteering],
                    SoftMaximum(dCorrectedPhi,-pWorld->m_Parameters[BulletVehicleParameters::MaxSteering],10),10);

    //steering needs to be flipped due to the way RayCastVehicle works
    dCorrectedPhi *= -1;

    //rate-limit the steering
    double dCurrentSteering = pWorld->vehicle_->GetAckermanSteering();
    double dRate = (dCorrectedPhi-dCurrentSteering)/dT;
    //clamp the rate
    dRate = sgn(dRate) * std::min(fabs(dRate),pWorld->m_Parameters[BulletVehicleParameters::MaxSteeringRate]);
    //apply the steering
    dCorrectedPhi = dCurrentSteering+dRate*dT;

    double wheelForce = dCorrectedForce/2;

    int wheelIndex = 2;
    pWorld->vehicle_->applyEngineForce(wheelForce,wheelIndex);
    wheelIndex = 3;
    pWorld->vehicle_->applyEngineForce(wheelForce,wheelIndex);

    wheelIndex = 0;
    //set the steering value
    pWorld->vehicle_->SetAckermanSteering(dCorrectedPhi);

    if (pWorld->m_pDynamicsWorld && bNoUpdate==false)
    {
        Eigen::Vector3d T_w = pWorld->state_.t_wv_.so3()*command.torque_;
        btVector3 bTorques(T_w[0],T_w[1], T_w[2]);
        pWorld->vehicle_->getRigidBody()->applyTorque(bTorques);
        //dout("Sending torque vector " << T_w.transpose() << " to car.");
        pWorld->m_pDynamicsWorld->stepSimulation(dT,1,dT);
    }


    //do this in a critical section
    {
        std::lock_guard<std::mutex> lock(*pWorld);
        //get chassis data from bullet
        Eigen::Matrix4d Twv;
        pWorld->vehicle_->getChassisWorldTransform().getOpenGLMatrix(Twv.data());
        pWorld->state_.t_wv_ = Sophus::SE3d(Twv);

        if(pWorld->state_.wheel_states_.size() != pWorld->vehicle_->getNumWheels()) {
            pWorld->state_.wheel_states_.resize(pWorld->vehicle_->getNumWheels());
            pWorld->state_.wheel_omegas_.resize(pWorld->vehicle_->getNumWheels());
        }
        for(size_t ii = 0; ii < pWorld->vehicle_->getNumWheels() ; ii++) {
            //vehicle_->updateWheelTransform(ii,true);
            pWorld->vehicle_->getWheelInfo(ii).m_worldTransform.getOpenGLMatrix(Twv.data());
            pWorld->state_.wheel_states_[ii] = Sophus::SE3d(Twv);
            pWorld->state_.wheel_omegas_[ii] = pWorld->vehicle_->getWheelInfo(ii).m_raycastInfo.m_isInContact;
        }

        //get the velocity
        pWorld->state_.vel_w_dot_ << pWorld->vehicle_->getRigidBody()->getLinearVelocity()[0], pWorld->vehicle_->getRigidBody()->getLinearVelocity()[1], pWorld->vehicle_->getRigidBody()->getLinearVelocity()[2];
        pWorld->state_.omega_w_dot_ << pWorld->vehicle_->getRigidBody()->getAngularVelocity()[0], pWorld->vehicle_->getRigidBody()->getAngularVelocity()[1], pWorld->vehicle_->getRigidBody()->getAngularVelocity()[2];

        //set the steering
        pWorld->state_.steering_cmd_ = pWorld->vehicle_->GetAckermanSteering();
    }
}

/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d BulletCarModel::GetVehicleLinearVelocity(int worldId)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    btVector3 v = pWorld->vehicle_->getRigidBody()->getLinearVelocity();
    Eigen::Vector3d dV;
    dV << v.x(), v.y(), v.z();
    return dV;
}

/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d BulletCarModel::GetVehicleAngularVelocity(int worldId)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    btVector3 v = pWorld->vehicle_->getRigidBody()->getAngularVelocity();
    Eigen::Vector3d dV;
    dV << v.x(), v.y(), v.z();
    return dV;
}

/////////////////////////////////////////////////////////////////////////////////////////

Eigen::Vector3d BulletCarModel::GetVehicleInertiaTensor(int worldId)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);

    btVector3 bVec = pWorld->vehicle_->getRigidBody()->getInvInertiaDiagLocal();
    Eigen::Vector3d res;
    for(int ii = 0 ; ii < 3 ; ii++) {
        res(ii) = (bVec[ii] == 0 ? 0 : 1/bVec[ii]);
    }
    return res;
}


/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::GetVehicleState(int worldId,VehicleState& stateOut)
{
    BulletWorldInstance* pWorld = GetWorldInstance(worldId);
    std::lock_guard<std::mutex> lock(*pWorld);
    stateOut = pWorld->state_;
    stateOut.t_wv_.translation() += GetBasisVector(stateOut.t_wv_,2)*
                                   (pWorld->m_Parameters[BulletVehicleParameters::SuspRestLength] +
                                    pWorld->m_Parameters[BulletVehicleParameters::WheelRadius]+
                                    pWorld->m_Parameters[BulletVehicleParameters::SuspConnectionHeight]-0.01);
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::SetStateNoReset( BulletWorldInstance *pWorld , const Sophus::SE3d& Twv)
{
    btTransform trans;
    trans.setFromOpenGLMatrix(Twv.matrix().data());
    pWorld->m_pCarChassis->setAngularVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setLinearVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setCenterOfMassTransform(trans);
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::SetState( int world_id,  const VehicleState& state )
{
    BulletWorldInstance *pWorld = GetWorldInstance(world_id);
    std::lock_guard<std::mutex> lock(*pWorld);
    //load the backup onto the vehicle
    pWorld->state_backup_.LoadState(pWorld->vehicle_);

    //set the wheel positions and contact
    for(size_t ii = 0; ii < state.wheel_states_.size() ; ii++) {
        //vehicle_->updateWheelTransform(ii,true);
        pWorld->vehicle_->getWheelInfo(ii).m_worldTransform.setFromOpenGLMatrix(state.wheel_states_[ii].data());
        pWorld->vehicle_->getWheelInfo(ii).m_raycastInfo.m_isInContact = state.wheel_omegas_[ii];
    }

    //update the parameters since they will have been overwritten
    _InternalUpdateParameters(pWorld);
    btVector3 vel(state.vel_w_dot_[0], state.vel_w_dot_[1], state.vel_w_dot_[2]);
    btVector3 w(state.omega_w_dot_[0], state.omega_w_dot_[1], state.omega_w_dot_[2]);

    //set the state 4x4 matrix, however offset the body up to account for the wheel columns
    Sophus::SE3d T = state.t_wv_;
    T.translation() -= GetBasisVector(T,2)*
                        (pWorld->m_Parameters[BulletVehicleParameters::SuspRestLength] +
                        pWorld->m_Parameters[BulletVehicleParameters::WheelRadius]+
                        pWorld->m_Parameters[BulletVehicleParameters::SuspConnectionHeight]-0.01);
    SetStateNoReset(pWorld,T);

    pWorld->state_ = state;
    pWorld->state_.t_wv_ = T;

    //set the linear velocity of the car
    pWorld->vehicle_->getRigidBody()->setLinearVelocity(vel);
    pWorld->vehicle_->getRigidBody()->setAngularVelocity(w);


    //set the steering
    pWorld->vehicle_->SetAckermanSteering(state.steering_cmd_);


    //raycast all wheels so they are correctly positioned
//    for (int i=0;i<pWorld->vehicle_->getNumWheels();i++)
//    {
//        WheelInfo& wheel = pWorld->vehicle_->getWheelInfo(i);
//        pWorld->vehicle_->rayCast(wheel);
//    }
}

/////////////////////////////////////////////////////////////////////////////////////////
btRigidBody*	BulletCarModel::_LocalCreateRigidBody(BulletWorldInstance *pWorld, double mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask)
{
    btAssert((!shape || shape->getShapeType() != INVALID_SHAPE_PROXYTYPE));

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0,0,0);
    if (isDynamic)
        shape->calculateLocalInertia(mass,localInertia);

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

    btRigidBody::btRigidBodyConstructionInfo cInfo(mass,myMotionState,shape,localInertia);

    btRigidBody* body = new btRigidBody(cInfo);
    body->setContactProcessingThreshold(BT_LARGE_FLOAT);

    pWorld->m_pDynamicsWorld->addRigidBody(body,group,mask);

    return body;
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_InitVehicle(BulletWorldInstance* pWorld, std::map<int, double>& parameters)
{
    pWorld->m_Parameters = parameters;

    //delete any previous collision shapes
    for(int ii = 0 ; ii < pWorld->m_vVehicleCollisionShapes.size() ; ii++) {
        delete pWorld->m_vVehicleCollisionShapes[ii];
    }
    pWorld->m_vVehicleCollisionShapes.clear();

    pWorld->vehicle_ChassisShape = new btBoxShape(btVector3(pWorld->m_Parameters[BulletVehicleParameters::WheelBase],pWorld->m_Parameters[BulletVehicleParameters::Width],pWorld->m_Parameters[BulletVehicleParameters::Height]));
    pWorld->m_vVehicleCollisionShapes.push_back(pWorld->vehicle_ChassisShape);

    /*btCompoundShape* compound = new btCompoundShape();
    pWorld->m_vVehicleCollisionShapes.push_back(compound);
    btTransform localTrans;
    localTrans.setIdentity();
    //localTrans effectively shifts the center of mass with respect to the chassis
    localTrans.setOrigin(btVector3(0,0,0));

    compound->addChildShape(localTrans,pWorld->vehicle_ChassisShape);*/

    btTransform tr;
    tr.setIdentity();
    tr.setOrigin(btVector3(0,0,0));
    btVector3 vWheelDirectionCS0(0,0,1); //wheel direction is z
    btVector3 vWheelAxleCS(0,1,0); //wheel axle in y direction

    if(pWorld->m_pCarChassis != NULL) {
        pWorld->m_pDynamicsWorld->removeRigidBody(pWorld->m_pCarChassis);
        delete pWorld->m_pCarChassis;
    }

    pWorld->m_pCarChassis = _LocalCreateRigidBody(pWorld,pWorld->m_Parameters[BulletVehicleParameters::Mass],tr,pWorld->vehicle_ChassisShape, COL_CAR,COL_NOTHING);//chassisShape);
    //pWorld->m_pCarChassis = _LocalCreateRigidBody(pWorld,pWorld->m_Parameters.m_dMass,tr,compound, COL_CAR,COL_GROUND);//chassisShape);

    /// create vehicle
    pWorld->vehicle_RayCaster = new btDefaultVehicleRaycaster(pWorld->m_pDynamicsWorld);

    if( pWorld->vehicle_ != NULL ) {
        pWorld->m_pDynamicsWorld->removeVehicle(pWorld->vehicle_);
        delete pWorld->vehicle_;
    }

    pWorld->m_Tuning.m_frictionSlip = pWorld->m_Parameters[BulletVehicleParameters::TractionFriction];
    pWorld->m_Tuning.m_suspensionCompression = pWorld->m_Parameters[BulletVehicleParameters::CompDamping];
    pWorld->m_Tuning.m_suspensionStiffness = pWorld->m_Parameters[BulletVehicleParameters::Stiffness];
    pWorld->m_Tuning.m_suspensionDamping = pWorld->m_Parameters[BulletVehicleParameters::ExpDamping];
    pWorld->m_Tuning.m_maxSuspensionForce = pWorld->m_Parameters[BulletVehicleParameters::MaxSuspForce];
    pWorld->m_Tuning.m_maxSuspensionTravelCm = pWorld->m_Parameters[BulletVehicleParameters::MaxSuspTravel]*100.0;

    pWorld->vehicle_ = new RaycastVehicle(pWorld->m_Tuning,pWorld->m_pCarChassis,pWorld->vehicle_RayCaster,&pWorld->m_pDynamicsWorld->getSolverInfo());
    pWorld->vehicle_->setCoordinateSystem(CAR_RIGHT_AXIS,CAR_UP_AXIS,CAR_FORWARD_AXIS);
    ///never deactivate the vehicle
    pWorld->m_pCarChassis->forceActivationState(DISABLE_DEACTIVATION);
    pWorld->m_pDynamicsWorld->addVehicle(pWorld->vehicle_);


    bool bIsFrontWheel=true;

    btVector3 connectionPointCS0(pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2,pWorld->m_Parameters[BulletVehicleParameters::Width]/2-(0.3*pWorld->m_Parameters[BulletVehicleParameters::WheelWidth]), pWorld->m_Parameters[BulletVehicleParameters::SuspConnectionHeight]);
    pWorld->vehicle_->adomegaheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[BulletVehicleParameters::SuspRestLength],pWorld->m_Parameters[BulletVehicleParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
    connectionPointCS0 = btVector3(pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2, -pWorld->m_Parameters[BulletVehicleParameters::Width]/2+(0.3*pWorld->m_Parameters[BulletVehicleParameters::WheelWidth]),pWorld->m_Parameters[BulletVehicleParameters::SuspConnectionHeight]);
    pWorld->vehicle_->adomegaheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[BulletVehicleParameters::SuspRestLength],pWorld->m_Parameters[BulletVehicleParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
    connectionPointCS0 = btVector3(-pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2,-pWorld->m_Parameters[BulletVehicleParameters::Width]/2+(0.3*pWorld->m_Parameters[BulletVehicleParameters::WheelWidth]), pWorld->m_Parameters[BulletVehicleParameters::SuspConnectionHeight]);
    bIsFrontWheel = false;
    pWorld->vehicle_->adomegaheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[BulletVehicleParameters::SuspRestLength],pWorld->m_Parameters[BulletVehicleParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);
    connectionPointCS0 = btVector3(-pWorld->m_Parameters[BulletVehicleParameters::WheelBase]/2,pWorld->m_Parameters[BulletVehicleParameters::Width]/2-(0.3*pWorld->m_Parameters[BulletVehicleParameters::WheelWidth]), pWorld->m_Parameters[BulletVehicleParameters::SuspConnectionHeight]);
    pWorld->vehicle_->adomegaheel(connectionPointCS0,vWheelDirectionCS0,vWheelAxleCS,pWorld->m_Parameters[BulletVehicleParameters::SuspRestLength],pWorld->m_Parameters[BulletVehicleParameters::WheelRadius],pWorld->m_Tuning,bIsFrontWheel);

    for (size_t i=0;i<pWorld->vehicle_->getNumWheels();i++)
    {
        WheelInfo& wheel = pWorld->vehicle_->getWheelInfo(i);
        wheel.m_rollInfluence = pWorld->m_Parameters[BulletVehicleParameters::RollInfluence];
        Sophus::SE3d wheelTransform(Sophus::SO3d(),
                                    Eigen::Vector3d(wheel.m_chassisConnectionPointCS[0],wheel.m_chassisConnectionPointCS[1],wheel.m_chassisConnectionPointCS[2] /*+ wheel.getSuspensionRestLength()/2*/));
        pWorld->m_vWheelTransforms.push_back(wheelTransform);
    }

    pWorld->vehicle_->SetDynamicFrictionCoefficient(pWorld->m_Parameters[BulletVehicleParameters::DynamicFrictionCoef]);
    pWorld->vehicle_->SetStaticSideFrictionCoefficient(pWorld->m_Parameters[BulletVehicleParameters::StaticSideFrictionCoef]);
    pWorld->vehicle_->SetSlipCoefficient(pWorld->m_Parameters[BulletVehicleParameters::SlipCoefficient]);
    pWorld->vehicle_->SetMagicFormulaCoefficients(pWorld->m_Parameters[BulletVehicleParameters::MagicFormula_B],
                                                    pWorld->m_Parameters[BulletVehicleParameters::MagicFormula_C],
                                                    pWorld->m_Parameters[BulletVehicleParameters::MagicFormula_E]);


    //reset all parameters
    //m_pCarChassis->setCenterOfMassTransform(btTransform::getIdentity());
    pWorld->m_pCarChassis->setLinearVelocity(btVector3(0,0,0));
    pWorld->m_pCarChassis->setAngularVelocity(btVector3(0,0,0));
    pWorld->m_pDynamicsWorld->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(pWorld->m_pCarChassis->getBroadphaseHandle(),pWorld->m_pDynamicsWorld->getDispatcher());
    if (pWorld->vehicle_)
    {
        pWorld->vehicle_->resetSuspension();
        for (size_t i=0;i<pWorld->vehicle_->getNumWheels();i++)
        {
            //synchronize the wheels with the (interpolated) chassis worldtransform
            pWorld->vehicle_->updateWheelTransform(i,true);
        }
    }


    pWorld->state_backup_.SaveState(pWorld->vehicle_);
}

/////////////////////////////////////////////////////////////////////////////////////////
std::vector<Sophus::SE3d> BulletCarModel::GetWheelTransforms(const int worldIndex){
    BulletWorldInstance *pWorld = GetWorldInstance(worldIndex);
    return pWorld->m_vWheelTransforms;
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::ResetCommandHistory(int worldId)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    std::lock_guard<std::mutex>lock(*pWorld);
    pWorld->previous_commands_.clear();
    pWorld->total_command_time_ = 0;
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::GetCommandHistory(int worldId,CommandList &previousCommandsOut)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    std::lock_guard<std::mutex> lock(*pWorld);
    previousCommandsOut = pWorld->previous_commands_;
}

/////////////////////////////////////////////////////////////////////////////////////////
CommandList&        BulletCarModel::GetCommandHistoryRef(int worldId)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    return pWorld->previous_commands_;
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::SetCommandHistory(const int& worldId, const CommandList &previousCommands)
{
    BulletWorldInstance *pWorld = GetWorldInstance(worldId);
    std::lock_guard<std::mutex> lock(*pWorld);
    //find out the total time of the commands
    pWorld->total_command_time_ = 0;
    for(const ControlCommand& command : previousCommands ){
        pWorld->total_command_time_ += command.timestep_;
    }

    pWorld->previous_commands_ = previousCommands;
}

/////////////////////////////////////////////////////////////////////////////////////////
void BulletCarModel::_InitWorld(BulletWorldInstance* pWorld, btCollisionShape *pGroundShape, btVector3 dMin, btVector3 dMax, bool centerMesh)
{
    //add this to the shapes
    pWorld->m_pTerrainShape = pGroundShape;
    pWorld->m_vCollisionShapes.push_back(pWorld->m_pTerrainShape);
    //pWorld->m_vCollisionShapes.push_back(groundShape);

    pWorld->m_pCollisionConfiguration = new btDefaultCollisionConfiguration();
    pWorld->m_pDispatcher = new btCollisionDispatcher(pWorld->m_pCollisionConfiguration);

    //btVector3 worldMin(pHeightMap->GetMinX(),pHeightMap->GetMinY(),dMin(2)-100);
    //btVector3 worldMax(pHeightMap->GetMaxX(),pHeightMap->GetMaxY(),dMax(2)+100);
    //btVector3 worldMin(-1000,-1000,-1000);
    //btVector3 worldMax(1000,1000,1000);
    pWorld->m_pOverlappingPairCache = new btAxisSweep3(dMin,dMax);
    pWorld->m_pConstraintSolver = new btSequentialImpulseConstraintSolver();
    pWorld->m_pDynamicsWorld = new btDiscreteDynamicsWorld(pWorld->m_pDispatcher,pWorld->m_pOverlappingPairCache,pWorld->m_pConstraintSolver,pWorld->m_pCollisionConfiguration);
    pWorld->m_pDynamicsWorld->setDebugDrawer(&pWorld->m_DebugDrawer);
    pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe || btIDebugDraw::DBG_FastWireframe);
    //pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawAabb);
    //pWorld->m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_NoDebug);


    //set the gravity vector
    pWorld->m_pDynamicsWorld->setGravity(btVector3(0,0,BULLET_MODEL_GRAVITY));

    btTransform tr;
    tr.setIdentity();
    if(centerMesh == true){
        tr.setOrigin(btVector3((dMax[0] + dMin[0])/2,(dMax[1] + dMin[1])/2,(dMax[2] + dMin[2])/2));
    }

    //create the ground object
    _LocalCreateRigidBody(pWorld,0,tr,pWorld->m_pTerrainShape,COL_GROUND,COL_RAY|COL_CAR);
    //_LocalCreateRigidBody(pWorld,0,tr,pWorld->m_pTerrainShape,COL_GROUND,COL_RAY|COL_CAR);

    //m_pHeightMap = pHeightMap;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool BulletCarModel::RayCast(const Eigen::Vector3d& dSource,const Eigen::Vector3d& dRayVector, Eigen::Vector3d& dIntersect, const bool& biDirectional, int index /*= 0*/)
{
    btVector3 source(dSource[0],dSource[1],dSource[2]);
    btVector3 vec(dRayVector[0],dRayVector[1],dRayVector[2]);
    btVector3 target = source + vec;
    BulletWorldInstance*pInstance = GetWorldInstance(index);

    btVehicleRaycaster::btVehicleRaycasterResult results,results2;
    if( biDirectional ){
        source = source - vec;
    }

    if(pInstance->vehicle_RayCaster->castRay(source,target,results) == 0){
        return false;
    }else{
        Eigen::Vector3d dNewSource(source[0],source[1],source[2]);
        dIntersect = dNewSource + results.m_distFraction* (biDirectional ? (Eigen::Vector3d)(dRayVector*2) : dRayVector);
        return true;
    }  
}
