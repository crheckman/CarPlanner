#include <CarPlanner/ApplyVelocitiesFunctor.h>
#include <CarPlanner/control_command.h>
#include <CarPlanner/vehicle_state.h>
#include <CarPlanner/vehicle_parameters.h>
#include <Eigen/StdVector>

static bool& g_bSkidCompensationActive(CVarUtils::CreateCVar("debug.SkidCompensationActive", false, ""));

////////////////////////////////////////////////////////////////
ApplyVelocitesFunctor5d::ApplyVelocitesFunctor5d(std::shared_ptr<carplanner::NinjaCar<Vehicle,Controller>> vehicle, Eigen::Vector3d init_torques, CommandList *pPreviousCommands /* = NULL */) :
    vehicle_(vehicle),
    init_torques_(init_torques),
    no_delay_(false)
{

    if(pPreviousCommands != NULL) {
        previous_commands_ = *pPreviousCommands;
    }
}

////////////////////////////////////////////////////////////////
double ApplyVelocitesFunctor5d::GetGravityCompensation(int current_index)
{
    double aExtra =  -(m_pCarModel->GetTotalGravityForce(m_pCarModel->GetWorldInstance(current_index))/m_pCarModel->GetWorldInstance(current_index)->m_Parameters[CarParameters::Mass])*1.1/*CAR_GRAVITY_COMPENSATION_COEFFICIENT*/;
    return aExtra;
}

////////////////////////////////////////////////////////////////
double ApplyVelocitesFunctor5d::GetFrictionCompensation(int current_index, double dt)
{
    double aExtra = -m_pCarModel->GetTotalWheelFriction(current_index,dt)/m_pCarModel->GetWorldInstance(current_index)->m_Parameters[CarParameters::Mass];
    return aExtra;
}

////////////////////////////////////////////////////////////////
//double ApplyVelocitesFunctor5d::GetSteeringCompensation(VehicleState& state, double phi, double curvature, int current_index)
//{
//    //get the corrected steering parameters
//    phi = m_pCarModel->GetCorrectedSteering(curvature,current_index);
//    double vf = state.vel_w_dot_.norm();
//    double aExtra = fabs(vf*vf*curvature*tan(phi))*CAR_STEERING_COMPENSATION_COEFFICIENT;
//    return aExtra;
//}

////////////////////////////////////////////////////////////////
void ApplyVelocitesFunctor5d::ApplyVelocities(const VehicleState& starting_state,
                                              std::vector<ControlCommand>& commands,
                                              std::vector<VehicleState>& states_out,
                                              const int start_index,
                                              const int end_index,
                                              const int current_index,
                                              const bool no_compensation /*= false*/,
                                              const CommandList *pPreviousCommands /*= NULL*/) {
    Eigen::Vector3d torques;
    Eigen::Vector4dAlignedVec vCoefs;
    BulletWorldInstance* pWorld = m_pCarModel->GetWorldInstance(current_index);

    vStatesOut.clear();

    double dTime = 0;

    VehicleState current_state;
    m_pCarModel->SetState(current_index,starting_state);
    m_pCarModel->VehicleState(current_index,current_state);
    VehicleState* current_state = &current_state; //this is necessary as we need to get a pointer to the current state for compensations
    //clear all the previous commands but chose between the member list or the one passed to the function
    m_pCarModel->SetCommandHistory(current_index, pPreviousCommands == NULL ? previous_commands_ : *pPreviousCommands);
    //m_pCarModel->ResetCommandHistory(current_index);

    vStatesOut.resize(nEndIndex-nStartIndex);

    ControlCommand command;
    for (int ii = nStartIndex; ii < nEndIndex; ii++) {
        //update the vehicle state
        //approximation for this dt
        command = vCommands[ii];
        if(no_compensation == false ){
            //HACK: SampleAcceleration actually returns this as acceleration and
            //not force, so we have to change that here
            double totalAccel = command.m_dForce;



            //compensate for gravity/slope
            double aExtra = 0;
            double dCorrectedCurvature;
            command.m_dPhi = m_pCarModel->GetSteeringAngle(command.curvature_,
                                                dCorrectedCurvature,current_index,1.0);

            //if(dRatio < 1.0){
            if(g_bSkidCompensationActive){
                //get the steering compensation
                std::pair<double,double> leftWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(current_index,0,command.m_dPhi,command.m_dT);
                std::pair<double,double> rightWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(current_index,1,command.m_dPhi,command.m_dT);
                double dRatio = std::max(fabs(leftWheel.second/leftWheel.first),fabs(rightWheel.second/rightWheel.first));

                for(int ii = 0 ; ii < 5 && dRatio < 1.0 ; ii++){
                    command.curvature_ *=1.5;///= (dRatio);
                    command.m_dPhi = m_pCarModel->GetSteeringAngle(command.curvature_,
                                                        dCorrectedCurvature,current_index,1.0);
                    std::pair<double,double> newLeftWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(current_index,0,command.m_dPhi,command.m_dT);
                    std::pair<double,double> newRightWheel = m_pCarModel->GetSteeringRequiredAndMaxForce(current_index,1,command.m_dPhi,command.m_dT);
                    dRatio = std::max(fabs(newLeftWheel.second/leftWheel.first),fabs(newRightWheel.second/rightWheel.first));
                }
            }



            aExtra += GetGravityCompensation(current_index);
            //aExtra += GetSteeringCompensation(*current_state,command.m_dPhi,command.curvature_,current_index);
            aExtra += GetFrictionCompensation(current_index,command.m_dT);


            totalAccel += aExtra;

//            if(dRatio < 1.0){
//                totalAccel = 0;//(dRatio*dRatio*dRatio);
//            }

            //actually convert the accel (up to this point) to a force to be applied to the car
            command.m_dForce = totalAccel*m_pCarModel->GetWorldInstance(current_index)->m_Parameters[CarParameters::Mass];
            //here Pwm = (torque+slope*V)/Ts
            command.m_dForce = sgn(command.m_dForce)* (fabs(command.m_dForce) + pWorld->m_Parameters[CarParameters::TorqueSpeedSlope]*pWorld->m_state.vel_w_dot_.norm())/pWorld->m_Parameters[CarParameters::StallTorqueCoef];
            command.m_dForce += pWorld->m_Parameters[CarParameters::AccelOffset]*SERVO_RANGE;

            //offset and coef are in 0-1 range, so multiplying by SERVO_RANGE is necessary
            command.m_dPhi = SERVO_RANGE*(command.m_dPhi*pWorld->m_Parameters[CarParameters::SteeringCoef] +
                                          pWorld->m_Parameters[CarParameters::SteeringOffset]);

            //save the command changes to the command array -- this is so we can apply
            //the commands to the vehicle WITH compensation
            vCommands[ii] = command;
        }

        //set the timestamp for this command
        vCommands[ii].timestamp_ = dTime;
        dTime += command.m_dT;
        m_pCarModel->UpdateState(current_index,command,command.m_dT,no_delay_);
        m_pCarModel->VehicleState(current_index,vStatesOut[ii-nStartIndex]);
        vStatesOut[ii-nStartIndex].curvature_ = command.curvature_;
        current_state = &vStatesOut[ii-nStartIndex];
        current_state->timestamp_ = dTime;

    }
}

////////////////////////////////////////////////////////////////
VehicleState ApplyVelocitesFunctor5d::ApplyVelocities(const VehicleState& startState,
                                                      MotionSample& sample,
                                                      int current_index /*= 0*/,
                                                      bool noCompensation /*= false*/) {
    ApplyVelocities(startState,
                    sample.m_vCommands,
                    sample.m_vStates,
                    0,
                    sample.m_vCommands.size(),
                    current_index,
                    noCompensation,
                    NULL);
    return sample.m_vStates.back();
}






