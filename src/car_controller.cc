#include <CarPlanner/car_controller.h>

static bool& g_bShow2DResult = CVarUtils::CreateGetUnsavedCVar("debug.Show2DResult",false);
static bool& g_optimize_2donly = CVarUtils::CreateGetUnsavedCVar("debug.Optimize2DOnly",false);
static bool& g_bForceZeroStartingCurvature = CVarUtils::CreateGetUnsavedCVar("debug.ForceZeroStartingCurvature",false);
static double& g_min_lookahead_time(CVarUtils::CreateGetUnsavedCVar("debug.MinLookaheadTime",(double)0.05,""));
static double& g_max_lookahead_time(CVarUtils::CreateGetUnsavedCVar("debug.MaxLookaheadTime",(double)2.0,""));
static double& g_initial_lookahead_time(CVarUtils::CreateGetUnsavedCVar("debug.InitialLookaheadTime",(double)0.5,""));
static double& g_max_plan_time_limit(CVarUtils::CreateGetUnsavedCVar("debug.MaxPlanTimeLimit",(double)1.0,""));
static double& g_lookahead_ema_weight(CVarUtils::CreateGetUnsavedCVar("debug.LookaheadEmaWeight",1.0,""));
static bool& g_freeze_control(CVarUtils::CreateGetUnsavedCVar("debug.FreezeControl",false,""));
static bool& g_point_cost(CVarUtils::CreateGetUnsavedCVar("debug.PointCost",false,""));
static bool& g_inertial_control = CVarUtils::CreateGetUnsavedCVar("debug.InertialControl",false);
static bool& g_bInfiniteTime = CVarUtils::CreateGetUnsavedCVar("debug.InfiniteTime",false);
static bool& g_bFrontFlip = CVarUtils::CreateGetUnsavedCVar("debug.FrontFlip",false);
static double& g_dMaxPlanNorm = CVarUtils::CreateGetUnsavedCVar("debug.MaxPlanNorm",5.0);


/////////////////////////////////////////////////////////////////////////////////////////
CarController::CarController() :
    g_max_control_plan_time(CVarUtils::CreateGetCVar("controller.MaxControlPlanTime",(float)0.2,"")),
    lookahead_time_(CVarUtils::CreateUnsavedCVar("controller.LookaheadTime",(float)0.2,"")),
    control_planner_thread_(NULL)
{
    last_delta_.setZero();
    //m_vControlPlans.reserve(10);
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::Init(std::vector<MotionSample>& segment_samples,
                         LocalPlanner *planner,
                         std::shared_ptr<carplanner::NinjaCar<Vehicle>> vehicle,
                         double dt)
{
    segment_samples_ = segment_samples;
    vehicle_ = vehicle;
    planner_ = planner;
    whether_stopping_ = false;
    whether_started_ = false;
    whether_first_pose_ = true;
    //m_pCurrentPlan = NULL;
    timestep_ = dt;
    state_updated_ = false;
}


/////////////////////////////////////////////////////////////////////////////////////////
void CarController::Reset()
{
    {
        std::lock_guard<std::mutex>(plan_mutex_);
        while(control_plans_list_.begin() != control_plans_list_.end()) {
            //delete this plan
            delete(control_plans_list_.front());
            control_plans_list_.erase(control_plans_list_.begin());
        }
    }

    last_command_ = ControlCommand();
    whether_first_pose_ = true;
    whether_stopping_ = false;
    whether_started_ = false;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool CarController::_SampleControlPlan(ControlPlan* plan,
                                       LocalProblem& problem)
{
    //get the motion sample for the new control plan
    if(g_optimize_2donly == true){
        plan->sample_.commands_vector_ = motion_sample_2donly_.commands_vector_;
    }else{
        plan->sample_.commands_vector_ = problem.best_solution_->sample_.commands_vector_;
    }

    if(g_bShow2DResult) {
        //ONLY FOR VISUALIZATION. REMOVE WHEN NO LONGER NEEDED
       Eigen::Vector3dAlignedVec samples;

        planner_->SamplePath(problem,samples,true);
        plan->sample_.states_vector_.reserve(samples.size());
        for(const Eigen::Vector3d& pos : samples){
            Sophus::SE3d Twv(Sophus::SO3d(),pos);
            plan->sample_.states_vector_.push_back(VehicleState(Twv,0));
        }
    }else{
        problem.functor_->ApplyVelocities(plan->start_state_,
                                          plan->sample_.commands_vector_,
                                          plan->sample_.states_vector_, 0,
                                          plan->sample_.commands_vector_.size(), 0,
                                          true);
        //if we are in the air, make sure no force is applied and the wheels are straight
        for(size_t ii = 0 ; ii < plan->sample_.states_vector_.size() ; ii++){
            if(plan->sample_.states_vector_[ii].IsAirborne()){
                if(g_bFrontFlip){
                    plan->sample_.commands_vector_[ii].force_ = 0;
                }else{
                    plan->sample_.commands_vector_[ii].force_ = 0 + problem.functor_->GetCarModel()->GetParameters(0)[VehicleParameters::AccelOffset]*SERVO_RANGE;
                }
                plan->sample_.commands_vector_[ii].phi_ = 0 + problem.functor_->GetCarModel()->GetParameters(0)[VehicleParameters::SteeringOffset]*SERVO_RANGE;
            }
        }
    }


    //get the plan times in order by offsetting them by the start time
    for(VehicleState& state: plan->sample_.states_vector_) {
        state.timestamp_ += plan->start_time_;
    }

    for(ControlCommand& command: plan->sample_.commands_vector_) {
        command.timestamp_ += plan->start_time_;
    }

    if(plan->sample_.commands_vector_.empty()) {
        dout("Empty control plan discovered...");
        return false;
    }

    plan->end_time_ = plan->sample_.states_vector_.back().timestamp_;
    //set the norm on the plan
    plan->norm_ = problem.current_solution_.norm_;

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool CarController::_SolveControlPlan(const ControlPlan* plan,
                                      LocalProblem& problem,
                                      const MotionSample& trajectory)
{
    bool res = planner_->InitializeLocalProblem(problem,
                                                plan->start_time_,
                                                &problem.velocity_profile_,
                                                g_point_cost ? eCostPoint : eCostTrajectory);
    problem.inertial_control_active_ = g_inertial_control;
    problem.trajectory_sample_ = trajectory;

    if( res == false ){
        dout("2d planner failed to converge...");
        return false;
    }

    res = true;

    std::chrono::high_resolution_clock::time_point timer =
        std::chrono::high_resolution_clock::now();

    while(1)
    {
        //make sure the plan is not fully airborne
        //bool isAirborne = (plan->start_state_.IsAirborne() && plan->goal_state_.IsAirborne());
        if(g_optimize_2donly /*|| isAirborne*/) {
            planner_->SimulateTrajectory(motion_sample_2donly_,problem,0,true);
            break;
        }else{
            if( (planner_->Iterate(problem)) == true ) {
                break;
            }
        }

        if(whether_stopping_){
            res = false;
        }

        std::chrono::high_resolution_clock::time_point now = std::chrono::high_resolution_clock::now();
        const double elapsed = std::chrono::duration_cast<std::chrono::microseconds>(timer - now).count();
        if(g_bInfiniteTime){
            if( elapsed > 1e6*g_max_plan_time_limit){
                break;
            }
        }else{
            if( elapsed > 1e6*(g_max_control_plan_time*lookahead_time_)){
                break;
            }
        }
    }

    //and now obtain the new delta
    last_delta_ = problem.current_solution_.optimization_params_ - problem.initial_optimization_params_;

    if(problem.current_solution_.norm_ > g_dMaxPlanNorm){
        dout("Planned control plan with norm too high -> " << problem.current_solution_.norm_  );
        res = false;
    }

    return res;
}

/////////////////////////////////////////////////////////////////////////////////////////
bool CarController::PlanControl(double plan_start_time, ControlPlan*& plan_out) {
    try
    {
        plan_out = NULL;
        int current_sample_index;
        double interpolation_amount;
        double plan_start_curvature;
        Eigen::Vector3d plan_start_torque = Eigen::Vector3d::Zero();
        PlanPtrList::iterator current_plan_index;
        ControlPlan* plan = NULL;

        //reset the starting position
        int start_segment_old_plan = 0;
        int start_sample_old_plan = 0;
        int sample_count = 0;

        //only continue planning if the pose has been updated since the last plan
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if(state_updated_ == false) {
                //dout("Pose not updated, exiting control.");
                return false;
            }else{
                state_updated_ = false;
            }
        }


        plan = new ControlPlan();

        {
            std::lock_guard<std::mutex> lock(plan_mutex_);

            //first find out where we are on the current plan
            _GetCurrentPlanIndex(plan_start_time,current_plan_index,current_sample_index,interpolation_amount);

            if(current_plan_index == control_plans_list_.end() ){
                //or if we have overshot all plans, clear
                while(control_plans_list_.begin() != control_plans_list_.end() )
                {
                    delete(control_plans_list_.front());
                    control_plans_list_.erase(control_plans_list_.begin());
                }
            }else{
                if(current_plan_index != control_plans_list_.begin() ){
                    //remove all plans before the current plan
                    while(control_plans_list_.begin() != current_plan_index) {
                        //delete this plan
                        delete(control_plans_list_.front());
                        control_plans_list_.erase(control_plans_list_.begin());
                    }
                    //the active plan should now be the first plan
                    current_plan_index = control_plans_list_.begin();
                }
            }
        }


        VehicleState current_state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_state = current_state_;
        }

//        ApplyVelocitesFunctor5d compdelay_functor(vehicle_,plan_start_torques, &current_commands_list_);
//        //compdelay_functor.reset_previous_commands();
//        compdelay_functor.set_no_delay(false);

//        //push the state forward by the duration of the solver if we have commmands
//        MotionSample compdelay_sample;
//        double commandTime = plan_start_time;
//        double maxTime =  plan_start_time + (g_max_control_plan_time*lookahead_time_);
//        while(commandTime < maxTime){
//            GetCurrentCommands(commandTime,last_command_);
//            last_command_.timestep_ = std::min(timestep_,maxTime - commandTime);
//            current_commands_list_.insert(current_commands_list_.begin(),last_command_);
//            compdelay_sample.commands_vector_.push_back(last_command_);
//            commandTime += timestep_;
//        }

//        if(compdelay_sample.commands_vector_.size() > 0){
//            compdelay_functor.ApplyVelocities(current_state,compdelay_sample,0,true);
//            current_state = compdelay_sample.states_vector_.back();
//        }

//        //also push forward the start time of this plan
//        plan_start_time += (g_max_control_plan_time*lookahead_time_);

        ApplyVelocitesFunctor5d delay_functor(vehicle_,plan_start_torque, NULL);
        //push forward the start state if there are commands stacked up
        MotionSample delay_sample;
        double total_delay = delay_functor.vehicle()->GetParams(0)[VehicleParameters::ControlDelay];
        if(total_delay > 0 && current_commands_list_.size() != 0){
            for(const ControlCommand& command: current_commands_list_){
                if(total_delay <= 0){
                    break;
                }
                ControlCommand delay_command = command;
                delay_command.timestep_ = std::min(total_delay,command.timestep_);
                delay_sample.commands_vector_.insert(delay_sample.commands_vector_.begin(),delay_command);
                total_delay -= delay_command.timestep_;
            }
            //delay_functor.reset_previous_commands();
            delay_functor.set_no_delay(true);
            //this applyvelocities call has whether_compensation set to true, as the commands
            //are from a previous plan which includes compensation
            delay_functor.ApplyVelocities(current_state,delay_sample,0,true);
            //and now set the starting state to this new value
            plan->start_state_ = delay_sample.states_vector_.back();
            last_command_ = delay_sample.commands_vector_.back();
        }else{
            Eigen::Vector3d target_velocity;
            Sophus::SE3d target_position;
            GetCurrentCommands(plan_start_time,last_command_,target_velocity,target_position);
            plan->start_state_ = current_state;
        }

        plan_start_torques = last_command_.torque_;
        plan_start_curvature = last_command_.curvature_;

        //dout("Plan starting curvature: " << plan_start_curvature);

        //double distanceToPath = 0;
        //if we do not have a plan, create new one from our
        //current position
        if(control_plans_list_.empty()){
            //get the starting curvature of our current plan

            //set the start time as now
            plan->start_time_ = plan_start_time;
            plan->start_segment_index_ = start_segment_old_plan;
            plan->start_sample_index_ = start_sample_old_plan;

            //start by finding the closest segment to our current location
            if(whether_first_pose_){
                //if this is the first pose, search everywhere for the car
                start_segment_old_plan = 0;
                start_sample_old_plan = 0;
                sample_count = 0;
                for(size_t jj = 0 ; jj < segment_samples_.size() ; jj++) {
                    sample_count += segment_samples_[jj].commands_vector_.size();
                }
                whether_first_pose_ = false;
                AdjustStartingSample(segment_samples_,
                                     plan->start_state_,
                                     plan->start_segment_index_,
                                     plan->start_sample_index_,
                                     0,
                                     sample_count);
            }else{
               AdjustStartingSample(segment_samples_,
                                    plan->start_state_,
                                    plan->start_segment_index_,
                                    plan->start_sample_index_);
            }

        }else {
            if(current_sample_index == -1) {
                //if we have overshot the current plan, function must be called again to create a new plan
                dout("Overshot plan.");
                return false;
            }else {
                //get the curvature at the end of the projection to have a smooth transition in steering
                plan->start_time_ = plan_start_time;

                plan->start_segment_index_ = (*current_plan_index)->start_segment_index_;
                //push forward the index by the precalculated amount
                plan->start_sample_index_ = (*current_plan_index)->start_sample_index_;// + current_sample_index;
                MotionSample::FixSampleIndexOverflow(segment_samples_,plan->start_segment_index_,plan->start_sample_index_);

                AdjustStartingSample(segment_samples_,plan->start_state_,plan->start_segment_index_,plan->start_sample_index_);
            }
        }

        if(g_bForceZeroStartingCurvature == true){
            plan_start_curvature = 0;
        }
        plan->start_state_.curvature_ = plan_start_curvature;

        MotionSample trajectory_sample;
        VelocityProfile profile;
        //prepare the trajectory ahead
        CarController::PrepareLookaheadTrajectory(segment_samples_,
                                                  plan,
                                                  profile,
                                                  trajectory_sample,
                                                  g_initial_lookahead_time);

        ApplyVelocitesFunctor5d functor(vehicle_,plan_start_torques, NULL);
        functor.set_no_delay(true);
        LocalProblem problem(&functor,
                             plan->start_state_,
                             plan->goal_state_,
                             timestep_);
        problem.start_torques_ = plan_start_torques;
        problem.current_solution_.min_trajectory_time_ = g_initial_lookahead_time;
        problem.velocity_profile_ = profile;


        //solve the control plan
        if( _SolveControlPlan(plan, problem, trajectory_sample) == false ) {
            //do not use the plan
            dout("Could not solve plan.");
            return false;
        }



        //only need to sample the planner if the plan is not airborne
        if( _SampleControlPlan(plan,problem) == false ) {
            dout("Failed to sample plan.");
            return false;
        }

        //double controlDelay = problem.functor_->GetCarModel()->GetParameters(0)[VehicleParameters::ControlDelay];
        double new_lookahead = std::max(std::min(problem.best_solution_->min_trajectory_time_, g_max_lookahead_time),g_min_lookahead_time);
        lookahead_time_ = g_lookahead_ema_weight*new_lookahead + (1-g_lookahead_ema_weight)*lookahead_time_;

        //dout("Planned control with norm " << problem.current_norm_ << " and starting curvature " << plan->start_state_.curvature_);
        plan->start_pose_ = plan->start_state_.t_wv_;
        plan->end_pose_ = plan->goal_state_.t_wv_;


        {
            std::lock_guard<std::mutex> lock(plan_mutex_);
            plan->plan_id_ = rand() % 10000;
            //dout("Created control plan id:" << plan->plan_id_ << " with starting torques: " << plan_start_torques.transpose() << "with norm " << planner_->GetCurrentNorm());
            control_plans_list_.push_back(plan);
        }

        //update the old plan segment and samples
        start_segment_old_plan = plan->start_segment_index_;
        start_sample_old_plan = plan->start_sample_index_;

        //make sure the pointer returned back is valid
        plan_out = plan;

        //do this so we create a new plan in the next iteration
        plan = NULL;


    }catch(...)
    {
        dout("Exception caught while planning.");
        return false;
    }
    return true;
}

VehicleState CarController::GetCurrentState() {
    VehicleState state_out;
    std::lock_guard<std::mutex> lock(state_mutex_);
    state_out = current_state_;
    std::cout << "state or pose? l. 444 car_controller.cc" << std::endl;
    return state_out;
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::SetCurrentPoseFromCarModel(std::shared_ptr<carplanner::NinjaCar<Vehicle>> vehicle, int world_id) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    //Sophus::SE3d oldTwv = current_state.t_wv_;
    vehicle->GetVehicleState(0,current_state_);
    //remove the car offset from the car state
    //current_state.t_wv_.block<3,1>(0,3) += current_state.t_wv_.block<3,1>(0,2)*CAR_HEIGHT_OFFSET;
    vehicle->GetCommandHistory(0,current_commands_list_);
    state_updated_ = g_freeze_control ? false : true;
    std::cout << "SetCurrentPoseFromCarModel l. 452 car_controller.cc" << std::endl;
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::SetCurrentPose(VehicleState state, CommandList* command_list /*= NULL*/) {
    std::lock_guard<std::mutex> lock(state_mutex_);

    if( std::isfinite(state.vel_w_dot_[0]) == false ){
        assert(false);
    }

    current_state = state;

    if(command_list != NULL) {
        current_commands_list_ = *command_list;
    }

    state_updated_ = true;
}

/////////////////////////////////////////////////////////////////////////////////////////
double CarController::GetLastPlanStartTime()
{
    std::lock_guard<std::mutex> lock(plan_mutex_);
    if(control_plans_list_.empty() == false){
        return control_plans_list_.back()->start_time_;
    }else{
        return -1;
    }
}


/////////////////////////////////////////////////////////////////////////////////////////
void CarController::GetCurrentCommands(const double time,
                                       ControlCommand& command)
{
    Eigen::Vector3d target_velocity;
    Sophus::SE3d dT_target;
    GetCurrentCommands(time,command,target_velocity,dT_target);
}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::GetCurrentCommands(const double time,
                                       ControlCommand& command,
                                       Eigen::Vector3d& target_velocity,
                                       Sophus::SE3d& dT_target)
{
    std::lock_guard<std::mutex> lock(plan_mutex_);
    int current_sample_index;
    PlanPtrList::iterator current_plan_index;
    double interpolationAmount;
    _GetCurrentPlanIndex(time,current_plan_index,current_sample_index,interpolationAmount);
    if( current_sample_index == -1 || current_plan_index == control_plans_list_.end() ) {
        //dout("GetCurrentCommands returning last commands a:" << m_dLastAccel << " c:" << m_dLastTurnRate << " t:" << m_dLastTorques.transpose());
        command.force_ = vehicle_->GetParameters(0)[VehicleParameters::AccelOffset]*SERVO_RANGE;
        command.phi_ = vehicle_->GetParameters(0)[VehicleParameters::SteeringOffset]*SERVO_RANGE;
        command.torque_ = Eigen::Vector3d::Zero();//m_dLastTorques;
        //dout("Torque output of: [ " << torques.transpose() << "] from previous plan");
    }else {
        command.force_ = (1-interpolationAmount) * (*current_plan_index)->sample_.commands_vector_[current_sample_index].force_ +
                            interpolationAmount * (*current_plan_index)->sample_.commands_vector_[current_sample_index+1].force_;

        command.phi_ =   (1-interpolationAmount) * (*current_plan_index)->sample_.commands_vector_[current_sample_index].phi_ +
                           interpolationAmount * (*current_plan_index)->sample_.commands_vector_[current_sample_index+1].phi_;


        command.curvature_ = (1-interpolationAmount) * (*current_plan_index)->sample_.commands_vector_[current_sample_index].curvature_ +
                                interpolationAmount * (*current_plan_index)->sample_.commands_vector_[current_sample_index+1].curvature_;

        command.torque_ = (1-interpolationAmount) * (*current_plan_index)->sample_.commands_vector_[current_sample_index].torque_ +
                            interpolationAmount * (*current_plan_index)->sample_.commands_vector_[current_sample_index+1].torque_;


        //dout("v: " << segment_samples_[(*current_plan_index)->start_segment_index_].states_vector_[(*current_plan_index)->start_sample_index_].vel_w_dot_.transpose());
        //calculate target values

        int currentSegIndex, current_sample_index;
        currentSegIndex = (*current_plan_index)->start_segment_index_;
        current_sample_index = (*current_plan_index)->start_sample_index_ + current_sample_index;
        MotionSample::FixSampleIndexOverflow(segment_samples_,currentSegIndex,current_sample_index);
        dT_target =  segment_samples_[currentSegIndex].states_vector_[current_sample_index].t_wv_;
        target_velocity = segment_samples_[currentSegIndex].states_vector_[current_sample_index].vel_w_dot_;

        //dout("GetCurrentCommands planid:" << (*current_plan_index)->plan_id_ << " sample index:" << current_sample_index << " returning interpolation with i:" << interpolationAmount << " a:" << accel << " c:" << curvature << " t:" << torques.transpose());

        last_command_.force_ = command.force_;
        last_command_.curvature_ = command.curvature_;
        last_command_.phi_ = command.phi_;
        last_command_.torque_ = command.torque_;
    }


}

/////////////////////////////////////////////////////////////////////////////////////////
void CarController::_GetCurrentPlanIndex(double currentTime, PlanPtrList::iterator& planIndex, int& sample_index, double& interpolationAmount) {
    interpolationAmount = 0;
    sample_index = -1;
    planIndex = control_plans_list_.end();
    bool bPlanValid = false;

    if(control_plans_list_.empty() == false) {
        //for(int ii = 0; ii < m_vControlPlans.size() ; ii++) {
        for(PlanPtrList::iterator it = control_plans_list_.begin() ; it != control_plans_list_.end() ; it++) {
            sample_index = 0;

            //only if the current time is within the bounds of the plan, will we go and search
            //for the exact command
            if(currentTime >= (*it)->sample_.commands_vector_.front().timestamp_ &&
               currentTime <= (*it)->sample_.commands_vector_.back().timestamp_ &&
               (*it)->sample_.commands_vector_.size() > 1){
                planIndex = it;
                for(size_t jj = 1; jj < (*it)->sample_.commands_vector_.size() ; jj++){
                    if(((*it)->sample_.commands_vector_[jj].timestamp_) >= currentTime){
                        bPlanValid = true; //the plan has not yet finished
                        if(sample_index != -1){
                            double prevTime = (*it)->sample_.commands_vector_[sample_index].timestamp_;
                            double nextTime = (*it)->sample_.commands_vector_[jj].timestamp_;
                            interpolationAmount = (currentTime - prevTime) /(nextTime-prevTime);
                        }
                        break;
                    }
                    sample_index = jj;
                }
            }
        }
    }

    if( bPlanValid == false ) {
        planIndex = control_plans_list_.end();
        sample_index = -1;
    }

    if( sample_index != m_nLastCurrentPlanIndex) {
        m_nLastCurrentPlanIndex = sample_index;
    }

}




////////////////////////////////////////////////////////////////
double CarController::AdjustStartingSample(const std::vector<MotionSample>& segment_samples,
                                           VehicleState& state,
                                           int& segment_index,
                                           int& sample_index,
                                           int lower_limit /*= 100*/,
                                           int upper_limit /*= 100*/)
{
    //move within a certain neighbourhood of the samples and see if you can find a minimum distance to the trajectory
    int current_segment_index = segment_index;
    int current_sample_index = sample_index;
    double minDistance = DBL_MAX;
    const VehicleState& current_state = segment_samples[current_segment_index].states_vector_[current_sample_index];
    Eigen::Vector3d distVect = current_state.t_wv_.translation() - state.t_wv_.translation();

    int min_segment_index = segment_index;
    int min_sample_index = sample_index;

    //offset by a certain amount before
    current_sample_index -= lower_limit; //0.5 seconds
    MotionSample::FixSampleIndexOverflow(segment_samples,current_segment_index,current_sample_index);

    for(int ii = 0; ii < upper_limit + lower_limit ; ii++){ //-0.5s -> +0.5s
        //fix any over/underflow
        MotionSample::FixSampleIndexOverflow(segment_samples,current_segment_index,current_sample_index);

        //see if this distance is less than the prevous
        const VehicleState& current_state2 = segment_samples[current_segment_index].states_vector_[current_sample_index];

        distVect = current_state2.t_wv_.translation() - state.t_wv_.translation();
        double sn = distVect.squarenorm();
        if( sn <= minDistance ) {
            minDistance = sn;
            min_segment_index = current_segment_index;
            min_sample_index = current_sample_index;
        }

        //increment the current sample
        current_sample_index++;
    }
    sample_index = min_sample_index;
    segment_index = min_segment_index;

    //return the minimum distance
    return minDistance;
}

////////////////////////////////////////////////////////////////
void CarController::PrepareLookaheadTrajectory(const std::vector<MotionSample> &segment_samples,
                                               ControlPlan *plan,
                                               VelocityProfile& trajectoryProfile,
                                               MotionSample& trajectory_sample,
                                               const double dLookaheadTime)
{
    double dLookahead = dLookaheadTime;
    //create a motion sample from this plan
    trajectoryProfile.push_back(VelocityProfileNode(0,plan->start_state_.vel_w_dot_.norm()));

    int seg = plan->start_segment_index_;
    int spl = plan->start_sample_index_;
    //reserve some states to improve efficiency
    trajectory_sample.states_vector_.reserve(segment_samples[seg].states_vector_.size());
    double trajTime = 0;
    while(trajTime <= dLookahead){
        trajectory_sample.states_vector_.push_back(segment_samples[seg].states_vector_[spl]);
        //set the correct time on the trajectory and increment
        trajectory_sample.states_vector_.back().timestamp_ = trajTime;
        trajTime += segment_samples[seg].commands_vector_[spl].timestep_;
        spl++;
        if(MotionSample::FixSampleIndexOverflow(segment_samples,seg,spl) && trajTime >= g_min_lookahead_time){
            trajectoryProfile.push_back(VelocityProfileNode(trajectory_sample.GetDistance(),trajectory_sample.states_vector_.back().vel_w_dot_.norm()));
        }

//        if(trajectory_sample.states_vector_.back().IsAirborne()){
//            dLookahead = std::max(dLookahead,trajTime);
//        }
    }

    const double totalDist = trajectory_sample.GetDistance();
    trajectoryProfile.push_back(VelocityProfileNode(totalDist,trajectory_sample.states_vector_.back().vel_w_dot_.norm()));
    for(VelocityProfileNode& node : trajectoryProfile){
        node.m_dDistanceRatio /= totalDist;
    }

    const int nTotalTrajSamples = trajectory_sample.states_vector_.size();
    //now add some more states to the end of the sample (for trajectory tracking)
    for(int jj = 0 ; jj < nTotalTrajSamples ; jj++){
        trajectory_sample.states_vector_.push_back(segment_samples[seg].states_vector_[spl]);
        //set the correct time on the trajectory and increment
        trajectory_sample.states_vector_.back().timestamp_ = trajTime;
        trajTime += segment_samples[seg].commands_vector_[spl].timestep_;
        spl++;
        MotionSample::FixSampleIndexOverflow(segment_samples,seg,spl);
    }



    plan->ending_segment_index_ = plan->start_segment_index_;
    plan->ending_sample_index_ = plan->start_sample_index_+nTotalTrajSamples;
    MotionSample::FixSampleIndexOverflow(segment_samples,plan->ending_segment_index_,plan->ending_sample_index_);
    plan->goal_state_ = segment_samples[plan->ending_segment_index_].states_vector_[plan->ending_sample_index_];

    if(plan->goal_state_.IsAirborne()){
        plan->goal_state_.AlignWithVelocityVector();
        plan->goal_state_.curvature_ = 0;
    }else{
        plan->goal_state_.curvature_ = segment_samples[plan->ending_segment_index_].commands_vector_[plan->ending_sample_index_].curvature_;
    }

    if(plan->start_state_.vel_w_dot_.norm() >= 0.5){
        plan->start_state_.AlignWithVelocityVector();
    }

}

