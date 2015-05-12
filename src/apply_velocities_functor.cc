#include <CarPlanner/apply_velocities_functor.h>
#include <CarPlanner/control_command.h>
#include <CarPlanner/vehicle_state.h>
#include <CarPlanner/vehicle_parameters.h>
#include <Eigen/StdVector>

static bool& g_bSkidCompensationActive(CVarUtils::CreateCVar("debug.SkidCompensationActive", false, ""));

////////////////////////////////////////////////////////////////
ApplyVelocitesFunctor5d::ApplyVelocitesFunctor5d(std::shared_ptr<carplanner::NinjaCar > vehicle, Eigen::Vector3d init_torques, CommandList *pPreviousCommands /* = NULL */) :
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
    double aExtra =  -(vehicle_->GetTotalGravityForce(vehicle_->GetWorldInstance(current_index))/vehicle_->GetWorldInstance(current_index)->m_Parameters[VehicleParameters::Mass])*1.1/*CAR_GRAVITY_COMPENSATION_COEFFICIENT*/;
    return aExtra;
}

////////////////////////////////////////////////////////////////
double ApplyVelocitesFunctor5d::GetFrictionCompensation(int current_index, double dt)
{
    double aExtra = -vehicle_->GetTotalWheelFriction(current_index,dt)/vehicle_->GetWorldInstance(current_index)->m_Parameters[VehicleParameters::Mass];
    return aExtra;
}

////////////////////////////////////////////////////////////////
//double ApplyVelocitesFunctor5d::GetSteeringCompensation(VehicleState& state, double phi, double curvature, int current_index)
//{
//    //get the corrected steering parameters
//    phi = vehicle_->GetCorrectedSteering(curvature,current_index);
//    double vf = state.vel_w_dot_.norm();
//    double aExtra = fabs(vf*vf*curvature*tan(phi))*CAR_STEERING_COMPENSATION_COEFFICIENT;
//    return aExtra;
//}

////////////////////////////////////////////////////////////////
void ApplyVelocitesFunctor5d::ApplyVelocities(const VehicleState& start_state,
                                              std::vector<ControlCommand>& commands,
                                              std::vector<VehicleState>& states_out,
                                              const int start_index,
                                              const int end_index,
                                              const int current_index,
                                              const bool no_compensation /*= false*/,
                                              const CommandList *pPreviousCommands /*= NULL*/) {
    Eigen::Vector3d torques;
    Eigen::Vector4dAlignedVec vCoefs;
    BulletWorldInstance* pWorld = vehicle_->GetWorldInstance(current_index);

    states_out.clear();

    double dTime = 0;

    VehicleState current_state;
    vehicle_->SetState(current_index,start_state);
    vehicle_->GetVehicleState(current_index,current_state_);
    VehicleState* current_state = &current_state; //this is necessary as we need to get a pointer to the current state for compensations
    //clear all the previous commands but chose between the member list or the one passed to the function
    vehicle_->SetCommandHistory(current_index, pPreviousCommands == NULL ? previous_commands_ : *pPreviousCommands);
    //vehicle_->ResetCommandHistory(current_index);

    states_out.resize(end_index-start_index);

    ControlCommand command;
    for (int ii = start_index; ii < end_index; ii++) {
        //update the vehicle state
        //approximation for this dt
        command = command_vector[ii];
        if(no_compensation == false ){
            //HACK: SampleAcceleration actually returns this as acceleration and
            //not force, so we have to change that here
            double total_accel = command.force_;



            //compensate for gravity/slope
            double aExtra = 0;
            double dCorrectedCurvature;
            command.phi_ =
                vehicle_->GetSteeringAngle(command.curvature_, dCorrectedCurvature,
                                           current_index, 1.0);

            //if(dRatio < 1.0){
            if(g_bSkidCompensationActive){
                //get the steering compensation
                std::pair<double,double> left_wheel =
                    vehicle_->GetSteeringRequiredAndMaxForce(current_index, 0,
                                                             command.phi_,
                                                             command.timestep_);
                std::pair<double,double> right_wheel =
                    vehicle_->GetSteeringRequiredAndMaxForce(current_index, 1,
                                                             command.phi_,
                                                             command.timestep_);
                double dRatio = std::max(fabs(left_wheel.second/left_wheel.first),
                                         fabs(right_wheel.second/right_wheel.first));

                for(int ii = 0 ; ii < 5 && dRatio < 1.0 ; ii++){
                    command.curvature_ *=1.5;///= (dRatio);
                    command.phi_ =
                        vehicle_->GetSteeringAngle(command.curvature_,
                                                   dCorrectedCurvature,
                                                   current_index, 1.0);
                    std::pair<double,double> new_left_wheel =
                        vehicle_->GetSteeringRequiredAndMaxForce(current_index, 0,
                                                                 command.phi_,
                                                                 command.timestep_);
                    std::pair<double,double> new_right_wheel =
                        vehicle_->GetSteeringRequiredAndMaxForce(current_index, 1,
                                                                 command.phi_,
                                                                 command.timestep_);
                    dRatio = std::max(fabs(new_left_wheel.second/left_wheel.first),
                                      fabs(new_right_wheel.second/right_wheel.first));
                }
            }



            aExtra += GetGravityCompensation(current_index);
            aExtra += GetFrictionCompensation(current_index,command.timestep_);


            total_accel += aExtra;

            //actually convert the accel (up to this point) to a force to be applied to the car
            command.force_ = total_accel*vehicle_->GetWorldInstance(current_index)->m_Parameters[VehicleParameters::Mass];
            //here Pwm = (torque+slope*V)/Ts
            command.force_ = sgn(command.force_) * (fabs(command.force_) +
                                pWorld->m_Parameters[VehicleParameters::TorqueSpeedSlope] *
                pWorld->state_.vel_w_dot_.norm())/pWorld->m_Parameters[VehicleParameters::StallTorqueCoef];
            command.force_ += pWorld->m_Parameters[VehicleParameters::AccelOffset]*SERVO_RANGE;

            //offset and coef are in 0-1 range, so multiplying by SERVO_RANGE is necessary
            command.phi_ = SERVO_RANGE*(command.phi_*pWorld->m_Parameters[VehicleParameters::SteeringCoef] +
                                          pWorld->m_Parameters[VehicleParameters::SteeringOffset]);

            //save the command changes to the command array -- this is so we can apply
            //the commands to the vehicle WITH compensation
            command_vector[ii] = command;
        }

        //set the timestamp for this command
        command_vector[ii].timestamp_ = dTime;
        dTime += command.timestep_;
        vehicle_->UpdateState(current_index, command, command.timestep_, no_delay_);
        vehicle_->GetVehicleState(current_index,states_out[ii-start_index]);
        states_out[ii-start_index].curvature_ = command.curvature_;
        current_state = &states_out[ii-start_index];
        current_state->timestamp_ = dTime;

    }
}

////////////////////////////////////////////////////////////////
VehicleState ApplyVelocitesFunctor5d::ApplyVelocities(const VehicleState& startState,
                                                      MotionSample& sample,
                                                      int current_index /*= 0*/,
                                                      bool whether_compensation /*= false*/) {
    ApplyVelocities(startState,
                    sample.commands_vector_,
                    sample.states_vector_,
                    0,
                    sample.commands_vector_.size(),
                    current_index,
                    whether_compensation,
                    NULL);
    return sample.states_vector_.back();
}






