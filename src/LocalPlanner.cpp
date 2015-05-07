#include <CarPlanner/utils/cvar_helpers.h>
#include <CarPlanner/solvers/local_planner.h>

using namespace carplanner;

static bool& g_use_central_differences = CVarUtils::CreateGetUnsavedCVar("debug.UseCentralDifferences",true);
static double& g_success_norm = CVarUtils::CreateGetUnsavedCVar("debug.SuccessNorm",0.01);
static double& g_time_target = CVarUtils::CreateGetUnsavedCVar("debug.TimeTarget",0.00);
static bool& g_disable_damping = CVarUtils::CreateGetUnsavedCVar("debug.DisableDamping",false);
static bool& g_monotonic_cost(CVarUtils::CreateGetUnsavedCVar("debug.MonotonicCost", true,""));
static bool& g_verbose(CVarUtils::CreateGetUnsavedCVar("debug.Verbose", false,""));
static bool& g_trajectory_cost(CVarUtils::CreateGetUnsavedCVar("debug.TrajectoryCost", true,""));
static int& g_nutrajectory_sample__cost_segments(CVarUtils::CreateGetUnsavedCVar("debug.TrajectoryCostSegments", 10,""));

struct ApplyCommandsThreadFunctor {
    ApplyCommandsThreadFunctor(LocalPlanner *planner,
                               LocalProblem& problem,
                               const int index,
                               Eigen::Vector6d& pose_out_,
                               Eigen::VectorXd& errorOut,
                               MotionSample& sample,
                               const bool solve_boundary = false) :
        planner_(planner),
        problem_(problem),
        index_(index),
        pose_out_(pose_out_),
        error_out_(errorOut),
        sample_(sample),
        solve_boundary_(solve_boundary)

    {}

    // this redefines the () operator for ApplyCommandThreadFunctor.
    // therefore, when we call ACTF(), it actually does the following:
    void operator()()
    {
        if(solve_boundary_){
            problem_.boundary_solver_->Solve(&problem_.boundary_problem_);
        }
        pose_out_ = planner_->SimulateTrajectory(sample_, problem_, index_);
        error_out_ = planner_->_CalculateSampleError(sample_,
                                                     problem_,
                                                     problem_.current_solution_.min_trajectory_time_);
    }

    LocalPlanner *planner_;
    LocalProblem& problem_;
    const int index_;
    Eigen::Vector6d& pose_out_;
    Eigen::VectorXd& error_out_;
    MotionSample& sample_;
    bool solve_boundary_;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

inline Eigen::VectorXd GetPointLineError(const Eigen::Vector6d& line1,const Eigen::Vector6d& line2, const Eigen::Vector6d& point, double& interpolation_factor)
{
    Eigen::Vector3d vAB = (line2.head(3) - line1.head(3));
    Eigen::Vector3d vAC = (point.head(3) - line1.head(3));
    double dAB = vAB.norm();
    double dProjection = vAC.dot(vAB.normalized());
    interpolation_factor = std::max(std::min(dProjection/dAB,1.0),0.0);

    //now calculate the interpolated value
    Eigen::Vector6d intVal = (1.0-interpolation_factor)*line1 + (interpolation_factor)*line2;
    intVal[3] = AngleWrap(intVal[3]);

    //and now we can calculate the error to the interpolated pose
    Eigen::VectorXd intError(TRAJ_UNIT_ERROR_TERMS);
    intError.head(3) = intVal.head(3) - point.head(3);
    intError[3] = AngleWrap(intVal[3] - point[3]);
    intError[4] = intVal[5] - point[5];
    return intError;
}

LocalPlanner::LocalPlanner() :
    thread_pool_(PLANNER_NUM_THREADS),
    eps_(CVarUtils::CreateUnsavedCVar("planner.Epsilon", 1e-6, "Epsilon value used in finite differences.")),
    point_weight_(CVarUtils::CreateUnsavedCVar("planner.PointCostWeights",Eigen::MatrixXd(1,1))),
    traj_weight_(CVarUtils::CreateUnsavedCVar("planner.TrajCostWeights",Eigen::MatrixXd(1,1))),
    plan_counter_(0)
{

    //weight matrix
    point_weight_ = Eigen::MatrixXd(POINT_COST_ERROR_TERMS,1);
    traj_weight_ = Eigen::MatrixXd(TRAJ_EXTRA_ERROR_TERMS+TRAJ_UNIT_ERROR_TERMS,1);
    point_weight_.setIdentity();
    traj_weight_.setIdentity();

    point_weight_(0) = XYZ_WEIGHT;
    point_weight_(1) = XYZ_WEIGHT;
    point_weight_(2) = XYZ_WEIGHT;
    point_weight_(3) = THETA_WEIGHT;
    point_weight_(4) = VEL_WEIGHT_POINT;

    traj_weight_(0) = XYZ_WEIGHT;
    traj_weight_(1) = XYZ_WEIGHT;
    traj_weight_(2) = XYZ_WEIGHT;
    traj_weight_(3) = THETA_WEIGHT;
    traj_weight_(4) = VEL_WEIGHT_TRAJ;
    traj_weight_(5) = TIME_WEIGHT;
    traj_weight_(6) = CURV_WEIGHT;
}

///////////////////////////////////////////////////////////////////////
void LocalPlanner::SamplePath(const LocalProblem& problem,
                              Eigen::Vector3dAlignedVec& samples,
                              bool best_solution /* = true */ )
{
    samples.clear();
    BezierBoundaryProblem boundary_problem = problem.boundary_problem_;
    Sophus::SE2d T_start(Sophus::SO2d(problem.start_pose_[3]),problem.start_pose_.head(2));
    samples.reserve(boundary_problem.m_vPts.size());
    for(const Eigen::Vector2d& pt : boundary_problem.m_vPts) {
        Eigen::Vector3d pos(pt[0],pt[1],0);
        pos.head(2) = T_start*pos.head(2);

        //now transform this into the proper 3d pose
        pos = problem.t_3d_inv_ * pos;
        samples.push_back(pos);
    }
}

///////////////////////////////////////////////////////////////////////
Eigen::Vector6d LocalPlanner::_Transform3dGoalPose(const VehicleState& state,
                                                   const LocalProblem& problem) const
{
    //also transfer this pose into the 3d projected space we are working on
    const Sophus::SE3d pose = problem.t_3d_ * state.t_wv_;
    Eigen::Vector6d untransformed_pose;
    untransformed_pose << pose.translation()[0],
                         pose.translation()[1],
                         pose.translation()[2],
                         atan2( pose.matrix()(1,0), pose.matrix()(0,0)),
                         0,
                         state.vel_w_dot_.norm();

    return _TransformGoalPose(untransformed_pose,problem);
}


///////////////////////////////////////////////////////////////////////
Eigen::Vector6d  LocalPlanner::_TransformGoalPose(const Eigen::Vector6d& goal_pose,
                                                  const LocalProblem& problem) const
{
    Eigen::Vector6d result;
    Eigen::Vector3d pt;
    pt << goal_pose[0], goal_pose[1], 1;
    pt = problem.t_inv_ * pt;
    result << pt[0], pt[1],goal_pose[2], rpg::AngleWrap( goal_pose[3] - problem.start_pose_[3] ), goal_pose[4], goal_pose[5];
    return result;
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd LocalPlanner::_GetTrajectoryError(const MotionSample& sample,
                                                  const Eigen::Vector6dAlignedVec& transformed_poses,
                                                  const Eigen::Vector6d& end_pose,
                                                  double& min_time) const
{
    Eigen::VectorXd error(TRAJ_UNIT_ERROR_TERMS);
    int nTrajSize = sample.states_vector_.size();
    Eigen::Vector6d min_pose, min_pose_before, min_pose_after;
    int min_index = -1;
    //now find the closes point in the trajectory to our current position
    double dMinDist = DBL_MAX;
    for(int ii = 0; ii < nTrajSize ; ii++){
        //find the closest point to our current location
        Eigen::Vector3d cartError = transformed_poses[ii].head(3) - end_pose.head(3);
        double norm = (cartError).norm();
        if(norm < dMinDist){
            min_time = sample.states_vector_[ii].timestamp_;
            min_pose = transformed_poses[ii];;
            min_pose_after = ii < (nTrajSize-1) ? transformed_poses[ii+1] : min_pose;
            min_pose_before = ii > 0 ? transformed_poses[ii-1] : min_pose;
            min_index = ii;
            dMinDist = norm;
        }
    }

    //reset the error
    error.setZero();

    //calculate the distance at the minimum
    error.head(3) = min_pose.head(3) - end_pose.head(3);
    error[3] = rpg::AngleWrap(min_pose[3] - end_pose[3]);
    error[4] = min_pose[5] - end_pose[5];

    //now calculate the distance on both sides and find the minimum
    double interpolation_factor;
    Eigen::VectorXd before_error;
    if(min_pose != min_pose_before){
        before_error = GetPointLineError(min_pose_before,min_pose,end_pose,interpolation_factor);
        if(before_error.head(3).norm() < error.head(3).norm()){
            min_time = (1.0-interpolation_factor)*sample.states_vector_[min_index-1].timestamp_ +
                interpolation_factor*sample.states_vector_[min_index].timestamp_;
            error = before_error;
        }
    }


    Eigen::VectorXd after_error;
    if(min_pose != min_pose_after){
        after_error = GetPointLineError(min_pose,min_pose_after,end_pose,interpolation_factor);
        if(after_error.head(3).norm() < error.head(3).norm()){
            min_time = (1.0-interpolation_factor)*sample.states_vector_[min_index].timestamp_ + interpolation_factor*sample.states_vector_[min_index+1].timestamp_;
            error = after_error;
        }
    }

    return error;
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd LocalPlanner::_GetWeightVector(const LocalProblem& problem)
{
    int errorVecSize = problem.cost_mode_ == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nutrajectory_sample__cost_segments+TRAJ_EXTRA_ERROR_TERMS;
    Eigen::VectorXd omega;
    Eigen::VectorXd traj_weights(errorVecSize);
    if(problem.cost_mode_ == eCostTrajectory){
        double factor = 1.0/((double)g_nutrajectory_sample__cost_segments);
        for(int ii = 0; ii < g_nutrajectory_sample__cost_segments ; ii++){
            traj_weights.segment(ii*TRAJ_UNIT_ERROR_TERMS,TRAJ_UNIT_ERROR_TERMS) = traj_weight_.col(0).head(TRAJ_UNIT_ERROR_TERMS)*factor;
            factor *= 2;
        }
        traj_weights.tail(TRAJ_EXTRA_ERROR_TERMS) = traj_weight_.col(0).tail(TRAJ_EXTRA_ERROR_TERMS);
        omega = traj_weights;
        //dout("Trjaectory weights are: " << traj_weights.transpose());
    }else{
        omega = point_weight_.col(0).head(POINT_COST_ERROR_TERMS);
    }
    return omega;
}

///////////////////////////////////////////////////////////////////////
double LocalPlanner::_CalculateErrorNorm(const LocalProblem& problem,
                                         const Eigen::VectorXd& dError)
{
    Eigen::VectorXd omega = _GetWeightVector(problem);
    Eigen::VectorXd error = dError;
    dout("error vector is " << error.transpose());
    error.array() *= omega.array();
    return error.norm();
}

///////////////////////////////////////////////////////////////////////
Eigen::VectorXd LocalPlanner::_CalculateSampleError(const MotionSample& sample,
                                                    LocalProblem& problem,
                                                    double& min_traj_time) const
{
    int errorVecSize = problem.cost_mode_ == eCostPoint ? POINT_COST_ERROR_TERMS : TRAJ_UNIT_ERROR_TERMS*g_nutrajectory_sample__cost_segments+TRAJ_EXTRA_ERROR_TERMS;
    Eigen::VectorXd error;
    if(sample.states_vector_.size() == 0 ){
        dout(problem.plan_id_ << ":Sample with size 0 detected. Aborting.");
        error = Eigen::VectorXd(errorVecSize);
        error.setOnes();
        error *= DBL_MAX;
        return error;
    }
    //get the normalized velocity
    VehicleState state = sample.states_vector_.back();
    Eigen::Vector3d omega_goal = problem.goal_state_.t_wv_.so3().inverse()* state.omega_w_dot_;
    if(state.IsAirborne()){
        state.AlignWithVelocityVector();
    }

    Eigen::Vector6d end_pose = _Transform3dGoalPose(state,problem);
    if(problem.cost_mode_ == eCostPoint){
        error = Eigen::VectorXd(errorVecSize);

        error.head(3) = problem.transformed_goal_.head(3) - end_pose.head(3);
        error[3] = rpg::AngleWrap(problem.transformed_goal_[3] - end_pose[3]);
        error[4] = problem.transformed_goal_[5] - end_pose[5];

    }else if(problem.cost_mode_ == eCostTrajectory){
        error =Eigen::VectorXd(errorVecSize);

        if(problem.transformed_trajectory_.empty()){
            int nTrajSize = problem.trajectory_sample_.states_vector_.size();
            problem.transformed_trajectory_.reserve(nTrajSize);
            //push back the transformed poses
            for(int ii = 0; ii < nTrajSize ; ii++){
                VehicleState tempState = problem.trajectory_sample_.states_vector_[ii];
                if(tempState.IsAirborne()){
                    tempState.AlignWithVelocityVector();
                }
                problem.transformed_trajectory_.push_back(_Transform3dGoalPose(tempState, problem));
            }
        }


        error.setZero();
        error.segment(TRAJ_UNIT_ERROR_TERMS*(g_nutrajectory_sample__cost_segments-1),TRAJ_UNIT_ERROR_TERMS)  = _GetTrajectoryError(problem.trajectory_sample_,
                                                                                                                         problem.transformed_trajectory_,
                                                                                                                         end_pose,
                                                                                                                         min_traj_time);

        //now that we have the minimum point, calculate trajectory error points along the way
        int counter = 0;
        int start_index = 0;

        for(double ii = min_traj_time/(double)g_nutrajectory_sample__cost_segments ;
                   ii < min_traj_time && counter < ((double)g_nutrajectory_sample__cost_segments-1) ;
                   ii += min_traj_time/(double)g_nutrajectory_sample__cost_segments ){
            //get the poses at this point in time
            Eigen::Vector6d poseSample = _Transform3dGoalPose(VehicleState::GetInterpolatedState(sample.states_vector_,start_index,ii,start_index),problem);

            double min_time;
            error.segment(TRAJ_UNIT_ERROR_TERMS*counter,TRAJ_UNIT_ERROR_TERMS) = _GetTrajectoryError(problem.trajectory_sample_,
                                                                                                     problem.transformed_trajectory_,
                                                                                                     poseSample,
                                                                                                    min_time);
            counter++;
        }
        error /= (double)g_nutrajectory_sample__cost_segments;


        //segment cost

        error[error.rows()-2] = g_time_target-min_traj_time;
        error[error.rows()-1] = state.vel_w_dot_.norm()*omega_goal[2] - problem.goal_state_.curvature_; // sample.GetBadnessCost();

    }
    return error;
}

///////////////////////////////////////////////////////////////////////
bool LocalPlanner::_CalculateJacobian(LocalProblem& problem,
                                      Eigen::VectorXd& dCurrentErrorVec,
                                      LocalProblemSolution& coordinate_descent,
                                      Eigen::MatrixXd& J)
{
    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");
    Eigen::VectorXd errors[OPT_DIM*2],dCurrentError; //debug
    Eigen::VectorXd errors5 = Eigen::VectorXd::Random(5,1); //debug
    errors[OPT_DIM*2] = errors5;
    std::vector<std::shared_ptr<LocalProblem > > vCubicProblems;
    std::vector<std::shared_ptr<ApplyCommandsThreadFunctor > > vFunctors;
    vCubicProblems.resize(OPT_DIM*2);
    vFunctors.resize(OPT_DIM*2);
    Eigen::Vector6d pPoses[OPT_DIM*2],dCurrentPose;

    const double dEps = eps_;// * problem.current_solution_.norm_;

    for( int ii = 0; ii < OPT_DIM; ii++ ){
        int plusIdx = ii*2, minusIdx = ii*2+1;
        vCubicProblems[plusIdx] = std::make_shared<LocalProblem>(problem);
        Eigen::VectorXd delta(OPT_DIM);
        delta.setZero();
        delta(ii) += dEps;
        vCubicProblems[plusIdx]->UpdateOptParams(vCubicProblems[plusIdx]->current_solution_.optimization_params_.head(OPT_DIM)+delta);
        vFunctors[plusIdx] = std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                                          *vCubicProblems[plusIdx],
                                                                          plusIdx,
                                                                          pPoses[plusIdx],
                                                                          errors[plusIdx],
                                                                          (vCubicProblems[plusIdx]->current_solution_.sample_),
                                                                          true);
        thread_pool_.enqueue(*vFunctors[plusIdx]);

        if(g_use_central_differences == true){
            vCubicProblems[minusIdx] = std::make_shared<LocalProblem>(problem);
            Eigen::VectorXd delta(OPT_DIM);
            delta.setZero();
            delta(ii) -= dEps;
            vCubicProblems[minusIdx]->UpdateOptParams(vCubicProblems[minusIdx]->current_solution_.optimization_params_.head(OPT_DIM)+delta);
            vFunctors[minusIdx] = std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                                               *vCubicProblems[minusIdx],
                                                                               minusIdx,
                                                                               pPoses[minusIdx],
                                                                               errors[minusIdx],
                                                                               (vCubicProblems[minusIdx]->current_solution_.sample_),
                                                                               true);
            thread_pool_.enqueue(*vFunctors[minusIdx]);
        }
    }

    std::shared_ptr<LocalProblem >currentProblem = std::make_shared<LocalProblem>(problem);
    std::shared_ptr<ApplyCommandsThreadFunctor > currentFunctor =
        std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                     *currentProblem,
                                                     OPT_DIM*2+1,
                                                     dCurrentPose,
                                                     dCurrentError,
                                                     (currentProblem->current_solution_.sample_),
                                                     true);
    thread_pool_.enqueue(*currentFunctor);


    //wait for all simulations to finish
    while(thread_pool_.busy_threads() > (thread_pool_.num_threads())){
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    dCurrentErrorVec = dCurrentError;

    std::shared_ptr<LocalProblem> pcoordinate_descent;
    double dBestNorm = DBL_MAX;

    for( int ii = 0; ii < OPT_DIM ; ii++ ){
        int plusIdx = ii*2, minusIdx = ii*2+1;
        double norm = _CalculateErrorNorm(*vCubicProblems[plusIdx],errors[plusIdx]);
        if(std::isfinite(norm)){
            if( norm < dBestNorm ) {
                dBestNorm = norm;
                pcoordinate_descent = (vCubicProblems[plusIdx]);
            }
        }


        if(g_use_central_differences == true){
            norm = _CalculateErrorNorm(*vCubicProblems[minusIdx],errors[minusIdx]);
            if(std::isfinite(norm)){
                if( norm < dBestNorm ) {
                    dBestNorm = norm;
                    pcoordinate_descent = (vCubicProblems[plusIdx]);
                }
            }
        }

        if(g_verbose){
            dout("Dimension " << ii << " norm " << norm << " error-> [" << errors[plusIdx].transpose().format(CleanFmt) << "] vs. ["  << dCurrentErrorVec.transpose().format(CleanFmt));
        }

        //now that we have all the error terms, we can set this column of the jacobians

        Eigen::VectorXd col = g_use_central_differences ? ((errors[plusIdx]) - (errors[minusIdx]))/(2.0*dEps) : ((errors[plusIdx]) - dCurrentErrorVec)/(dEps);

        J.col(ii) = -col;
        //if this term is NAN, sound the alarm
        if(std::isfinite(col[0]) == false || std::isfinite(col[1]) == false ||
           std::isfinite(col[2]) == false || std::isfinite(col[3]) == false){
            problem.planner_error_ = eJacobianColumnNan;
            return false;
        }

    }

    coordinate_descent = pcoordinate_descent->current_solution_;
    coordinate_descent.norm_ = dBestNorm;


    if(g_verbose){
        dout("Jacobian:" << J.format(CleanFmt) << std::endl);
    }
    return true;
}


///////////////////////////////////////////////////////////////////////
double LocalPlanner::_DistanceTraveled( const double& t,
                                        const AccelerationProfile& profile ) const
{
    double totalDist = 0;
    double lastTime = 0;
    //go through each profile segment until we find the one that we're in
    for(size_t ii = 0 ; ii < profile.size() ; ii++){
        if(profile[ii].end_time_ < t && ii != profile.size()-1){
            totalDist += profile[ii].end_dist_;
            lastTime = profile[ii].end_time_;
            continue;
        }else{
            double dt = t - lastTime;
            totalDist += profile[ii].vel_dot_start_ * dt + (profile[ii].vel_dot_end_ - profile[ii].vel_dot_start_)*(dt * dt) / (2.0 * (profile[ii].end_time_-lastTime));
            break;
        }
    }
    return totalDist;
}

///////////////////////////////////////////////////////////////////////
void LocalPlanner::SampleAcceleration(std::vector<ControlCommand>& command_vector,
                                      LocalProblem& problem) const
{
    command_vector.clear();

    _GetAccelerationProfile(problem);
    if(std::isfinite(problem.segment_time_) == false ){
        dout(problem.plan_id_ << ":Segment time of " << problem.segment_time_ << " was not finite.");
        return;
    }
    double t;
    problem.segment_time_ = std::min(problem.segment_time_,problem.max_segment_time_);
    int numSamples = (int)(problem.segment_time_/problem.timestep_ + 1.0);
    command_vector.reserve(numSamples);

    size_t accelIndex = 0;
    for( t = 0; t < (problem.segment_time_) ; t+= problem.timestep_  ){
        double curvature = problem.boundary_solver_->GetCurvature(&problem.boundary_problem_,
                                                                   _DistanceTraveled(t,problem.accel_profile_));

        double step = problem.segment_time_ - t;
        double actualDt = std::min(problem.timestep_,step);

        if(problem.accel_profile_[accelIndex].end_time_ < t){
            accelIndex++;
        }

        if(accelIndex >= problem.accel_profile_.size()){
            dout(problem.plan_id_ << ":Exceeded bounds of acceleration profile.");
            return;
        }

        //if needed, add torques
        double endTime = problem.torque_start_time_ + problem.torque_duration_;
        Eigen::Vector3d dTorques = Eigen::Vector3d::Zero();
        if(t >= problem.torque_start_time_ && problem.torque_start_time_ != -1 && t <= (endTime)){
            dTorques(1) = problem.coefs_(0) + problem.coefs_(1)*(t-problem.torque_start_time_) +
                          problem.coefs_(2)*powi((t-problem.torque_start_time_),2) +
                          problem.coefs_(3)*powi((t-problem.torque_start_time_),3);
        }
        command_vector.push_back(ControlCommand(problem.accel_profile_[accelIndex].accel_+(problem.current_solution_.optimization_params_[OPT_ACCEL_DIM]/problem.segment_time_),curvature,dTorques,actualDt,0));
    }

}


///////////////////////////////////////////////////////////////////////
Eigen::Vector6d LocalPlanner::SimulateTrajectory(MotionSample& sample,
                                                 LocalProblem& problem,
                                                 const int nIndex /*= 0*/,
                                                 const bool& best_solution /* = false */)
{
    sample.Clear();
    bool bUsingBestSolution = false;
    if(best_solution && problem.best_solution_ != NULL && problem.best_solution_->sample_.commands_vector_.size() != 0){
        bUsingBestSolution = true;
        sample.commands_vector_ = problem.best_solution_->sample_.commands_vector_;
    }else{
        SampleAcceleration(sample.commands_vector_, problem);
    }

    VehicleState vState;
    if(sample.commands_vector_.size() == 0){
        vState = problem.start_state_;
    }else {
        vState = problem.functor_->ApplyVelocities( problem.start_state_, sample, nIndex, bUsingBestSolution);
    }
    //transform the result back
    Eigen::Vector6d dRes = _Transform3dGoalPose(vState,problem);
    return dRes;
}

///////////////////////////////////////////////////////////////////////
void LocalPlanner::_GetAccelerationProfile(LocalProblem& problem) const
{
    double totalDist = problem.boundary_problem_.m_dDistance;
    double currentDist = 0;
    double totalTime = 0;

    //must have at least two nodes
    assert(problem.velocity_profile_.size()>1);

    //prepare the accel profile
    problem.accel_profile_.clear();
    problem.accel_profile_.reserve(problem.velocity_profile_.size());

    //first calcualte the segment time
    for(size_t ii = 1; ii < problem.velocity_profile_.size(); ii++){
        //calculate the distance in this segment
        double segDist = (problem.velocity_profile_[ii].m_dDistanceRatio - problem.velocity_profile_[ii-1].m_dDistanceRatio)*totalDist;
        double segTime = segDist / (problem.velocity_profile_[ii-1].vel_w_dot_el + 0.5 * (problem.velocity_profile_[ii].vel_w_dot_el - problem.velocity_profile_[ii-1].vel_w_dot_el));
        totalTime += segTime;
        currentDist += segDist;

        //push back the accel profile
        double accel = (problem.velocity_profile_[ii].vel_w_dot_el - problem.velocity_profile_[ii-1].vel_w_dot_el)/segTime;
        problem.accel_profile_.push_back(AccelerationProfileNode(totalTime,accel,currentDist,problem.velocity_profile_[ii-1].vel_w_dot_el,problem.velocity_profile_[ii].vel_w_dot_el));
    }
    problem.segment_time_ = totalTime;
}

///////////////////////////////////////////////////////////////////////
bool LocalPlanner::InitializeLocalProblem(LocalProblem& problem,
                                          const double start_time,
                                          const VelocityProfile* velocity_profile /* = NULL*/,
                                          LocalProblemCostMode eCostMode /*= eCostPoint */)
{
    problem.Reset();
    problem.plan_id_ = plan_counter_++;

    //if there are previous commands, apply them so we may make a more educated guess
    MotionSample delay_sample;
    double total_delay = problem.functor_->GetCarModel()->GetParameters(0)[CarParameters::ControlDelay];
    if(total_delay > 0 && problem.functor_->previous_Command().size() != 0){
        CommandList::iterator it  = problem.functor_->previous_Command().begin();
        while(total_delay > 0 && it != problem.functor_->previous_Command().end()){
            delay_sample.commands_vector_.insert(delay_sample.commands_vector_.begin(),(*it));
            total_delay -= (*it).timestep_;
            ++it;
        }
        problem.functor_->Reset_previous_Commands();
        problem.functor_->set_no_delay(true);
        problem.functor_->ApplyVelocities(problem.start_state_,delay_sample,0,true);
        //and now set the starting state to this new value
        problem.start_state_ = delay_sample.states_vector_.back();
    }

    //regardless of the delay, for local planning we always want to proceed with no delay and with no previous commands
    //as the previous section should take care of that
    problem.functor_->Reset_previous_Commands();
    problem.functor_->set_no_delay(true);

    Sophus::SE3d dTranslation(Sophus::SO3d(),-problem.start_state_.t_wv_.translation());

    //first translate to base everything around dStartPose
    Sophus::SE3d dFixedStart = problem.start_state_.t_wv_;
    Sophus::SE3d dFixedGoal = problem.goal_state_.t_wv_;

    //now rotate everything to a frame inbetween the two
    Eigen::Quaternion<double> dStartQuat = problem.start_state_.t_wv_.so3().unit_quaternion();
    Eigen::Quaternion<double> dEndQuat = problem.goal_state_.t_wv_.so3().unit_quaternion();
    //find the halfway rotation
    Eigen::Quaternion<double> dMidQuat = dStartQuat.slerp(0.5,dEndQuat);

    //now rotate both waypoints into this intermediate frame
    Sophus::SE3d dRotation(Sophus::SO3d(dMidQuat.toRotationMatrix().transpose()),Eigen::Vector3d::Zero()); //we want the inverse rotation here
    static bool& bFlatten2Dcurves = CVarUtils::CreateGetCVar("debug.flatten2Dcurves",false,"");
    if(bFlatten2Dcurves){
        dRotation.so3() = Sophus::SO3d();
    }

    problem.t_3d_ = (dRotation*dTranslation);
    problem.t_3d_inv_ = problem.t_3d_.inverse();

    //now rotate both points;
    dFixedStart = problem.t_3d_ * dFixedStart;
    dFixedGoal = problem.t_3d_ * dFixedGoal;

    VehicleState dStartStateFixed = problem.start_state_;
    VehicleState dGoalStateFixed = problem.goal_state_;
    dStartStateFixed.t_wv_ = dFixedStart;
    dGoalStateFixed.t_wv_ = dFixedGoal;
    //and now we can project to 2d and get our 2d planner points
    Eigen::Vector6d dStartPose2D = dStartStateFixed.ToPose();
    Eigen::Vector6d goal_pose2D = dGoalStateFixed.ToPose();

    //make sure we don't have start and end velocities of zero
    if(dStartPose2D[5] == 0 && goal_pose2D[5] == 0){
        return false;
    }

    problem.start_time_ = start_time;

    //setup the boundary problem
    problem.start_pose_ = dStartPose2D;
    problem.m_goal_pose_ = goal_pose2D;
    problem.boundary_solver_ = &boundary_solver_;

    problem.t_inv_ = rpg::TInv( rpg::Cart2T( problem.start_pose_[0], problem.start_pose_[1], problem.start_pose_[3] ));
    Eigen::Vector6d dTemp = _TransformGoalPose(dStartPose2D,problem);
    problem.boundary_problem_.start_pose_ = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]);
    dTemp  = _TransformGoalPose(goal_pose2D,problem);
    problem.boundary_problem_.m_goal_pose_ = Eigen::Vector4d(dTemp[0],dTemp[1],dTemp[3],dTemp[4]);
    problem.boundary_problem_.m_nDiscretization = 50;

    if(velocity_profile == NULL){
        problem.velocity_profile_.clear();
        problem.velocity_profile_.reserve(2);
        problem.velocity_profile_.push_back(VelocityProfileNode(0,problem.start_pose_[5]));
        problem.velocity_profile_.push_back(VelocityProfileNode(1,problem.m_goal_pose_[5]));
    }else{
        problem.velocity_profile_ = *velocity_profile;
    }



    //this puts the result in m_dP
    problem.boundary_solver_->Solve(&problem.boundary_problem_);
    _GetAccelerationProfile(problem);

    //initialize the max time
    problem.max_segment_time_ = problem.segment_time_*5;

    //reset optimization parameters
    problem.current_solution_.norm_ = DBL_MAX;
    problem.in_local_minimum_ = false;
    problem.current_solution_.optimization_params_.head(3) = problem.boundary_problem_.m_goal_pose_.head(3);
    if(OPT_DIM > OPT_AGGR_DIM){
        problem.current_solution_.optimization_params_[OPT_AGGR_DIM] = problem.boundary_problem_.aggressiveness_;
    }
    problem.current_solution_.optimization_params_[OPT_ACCEL_DIM] = 0.0; // this is the percentage of the calculated acceleration
    problem.initial_optimization_params_ = problem.current_solution_.optimization_params_;
    problem.best_solution_ = &problem.current_solution_;

    //the transformed final position
    problem.transformed_goal_ = _TransformGoalPose(problem.m_goal_pose_, problem);

    //restore the original start state for simulation purposes
    problem.cost_mode_ = eCostMode;

    return true;
}


///////////////////////////////////////////////////////////////////////
bool LocalPlanner::Iterate(LocalProblem &problem )
{
    try
    {
        if(problem.current_solution_.norm_ < g_success_norm) {
            dout(problem.plan_id_ << ":Succeeded to plan. Norm = " << problem.current_solution_.norm_);
            return true;
        }

        //get the current state and norm for the first iteration
        if(problem.local_solutions_.size() == 0 ) {
            //here we have to re-evaluate the segment time of the trajectory

            SimulateTrajectory(problem.current_solution_.sample_,problem,0,false);
            if(problem.inertial_control_active_){
                CalculateTorqueCoefficients(problem,&problem.current_solution_.sample_);
                SimulateTrajectory(problem.current_solution_.sample_,problem,0,false);
            }

            //calculate the distance
            double dMinLookahead;
            Eigen::VectorXd error = _CalculateSampleError(problem,dMinLookahead);
            problem.current_solution_.norm_ = _CalculateErrorNorm(problem,error);
            if(std::isfinite(problem.current_solution_.norm_) == false){
                dout(problem.plan_id_ << ":Initial norm is not finite. Exiting optimization");
                return false;
            }
            problem.local_solutions_.push_back(problem.current_solution_);
            problem.best_solution_ = &problem.local_solutions_.back();
        }

        if( eps_ > 5 || problem.in_local_minimum_ == true) {
            return true;
        }

        _IterateGaussNewton(problem);
        if(problem.inertial_control_active_){
            CalculateTorqueCoefficients(problem,&problem.current_solution_.sample_);
        }
        return false;
    }catch(...){
        return false;
    }
}

void LocalPlanner::CalculateTorqueCoefficients(LocalProblem& problem,
                                               MotionSample* motion_sample)
{
    //find the last air section and record its duration
    double air_time = 0;
    int start_index = -1;
    double start_time = 0;

    double dLongestAirTime = 0, dLongestStartTime = 0;
    int nLongestStartIndex = -1;

    //find the longest airborne segment in the trajectory
    for(int ii = std::min(motion_sample->states_vector_.size()-1,
                          motion_sample->commands_vector_.size()-1) ;
            ii>=0 ; --ii){
        if(motion_sample->states_vector_[ii].IsAirborne()){
            air_time += motion_sample->commands_vector_[ii].timestep_;
            start_index = ii;
            start_time = motion_sample->states_vector_[ii].timestamp_;
        }else{
            if(air_time != 0){
                if(air_time > dLongestAirTime){
                    dLongestAirTime = air_time;
                    dLongestStartTime = start_time;
                    nLongestStartIndex = start_index;

                    air_time = 0;
                    start_index = -1;
                    start_time = 0;
                }
            }
        }
    }

    if(nLongestStartIndex != -1){
        air_time = dLongestAirTime;
        start_time = dLongestStartTime;
        start_index = nLongestStartIndex;
    }

    //now calcualte the coefficients for this maneuver based on the angle error
    if(air_time > 0.05){
        //find the goal state based on the cost type
        Sophus::SO3d dGoalState;
        if(problem.cost_mode_ == eCostPoint){
            dGoalState = problem.goal_state_.t_wv_.so3();
        }else{
            dGoalState = problem.goal_state_.t_wv_.so3();

            bool bCurrentAirborne = problem.trajectory_sample_.states_vector_[0].IsAirborne();
            for(size_t ii = 1 ; ii < problem.trajectory_sample_.states_vector_.size() ; ii++){
                bool bTemp = problem.trajectory_sample_.states_vector_[ii].IsAirborne();
                if(bTemp == false && bCurrentAirborne == true){
                    dGoalState = problem.trajectory_sample_.states_vector_[ii].t_wv_.so3();
                    break;
                }
                bCurrentAirborne = bTemp;
            }
        }
        //calculate the change in angles necessary
        const Sophus::SO3d Rw_dest = problem.goal_state_.t_wv_.so3();
        const Sophus::SO3d Rw_init = motion_sample->states_vector_[start_index].t_wv_.so3();

        //angle in body frame
        const Eigen::Vector3d angles(0,
                                     rpg::AngleWrap(R2Cart(Rw_dest.matrix())[1]-R2Cart(Rw_init.matrix())[1]),// - (problem.cost_mode_ == eCostPoint ? M_PI*2 : 0),
                                     0);// = Rinit_dest.log();

        const Eigen::Vector3d moment_of_inertia = problem.functor_->GetCarModel()->VehicleInertiaTensor(0);
        const Sophus::SO3d Rwv = motion_sample->states_vector_[start_index].t_wv_.so3();
        //angular velocities in body frame
        const Eigen::Vector3d omega_v = Rwv.inverse() * motion_sample->states_vector_[start_index].omega_w_dot_;

        //calculate the coefficients
        Eigen::Matrix4d A;
        Eigen::Vector4d B;

        A << 1, 0              ,0                     ,0,
                1, air_time, powi(air_time,2), powi(air_time,3),
                air_time, powi(air_time,2)/2,powi(air_time,3)/3,powi(air_time,4)/4,
                powi(air_time,2)/2,  powi(air_time,3)/6,powi(air_time,4)/12,powi(air_time,5)/20;

            Eigen::Matrix4d Ainertia = A / moment_of_inertia(1);
            B << problem.start_torques_(1),0, -omega_v(1), angles(1) - omega_v(1)*air_time;
            problem.coefs_ = Ainertia.inverse() * B;
            problem.torque_start_time_ = start_time;
            problem.torque_duration_ = air_time;
    }else{
        problem.torque_start_time_ = -1;
    }
}

///////////////////////////////////////////////////////////////////////
bool LocalPlanner::_IterateGaussNewton( LocalProblem& problem )
{
    Eigen::IOFormat CleanFmt(8, 0, ", ", "\n", "[", "]");
    Eigen::VectorXd dDeltaP;
    double bestDamping;
    unsigned int nBestDampingIdx = 0;

    Eigen::VectorXd omegavec = _GetWeightVector(problem);

    Eigen::MatrixXd J = Eigen::MatrixXd(omegavec.rows(),OPT_DIM);
    //create an appropriate weighting matrix
    Eigen::MatrixXd omega = omegavec.asDiagonal();

    //double dMinLookahead;
    Eigen::VectorXd error;// = _CalculateSampleError(problem,dMinLookahead);

    LocalProblemSolution coordinate_descent;

    std::cout << J << std::endl; //debug
    if(_CalculateJacobian(problem,error,coordinate_descent,J) == false){
        return false;
    }

    problem.current_solution_.norm_ = _CalculateErrorNorm(problem,error);
    if(g_verbose){
        dout("Calculated jacobian with base norm: " << problem.current_solution_.norm_);
    }

    //if any of the columns are zero, reguralize
    bool bZeroCols = false;
    for(int ii = 0 ; ii < J.cols() ; ii++){
        if( J.col(ii).norm() == 0){
            bZeroCols = true;
            break;
        }
    }


    //solve for the dDeltaP
    dDeltaP = J.transpose()*omega*error;
    Eigen::MatrixXd JtJ = J.transpose()*omega*J;
    //Do thikonov reguralization if there is a null column
    if(bZeroCols) {
        JtJ += Eigen::Matrix<double,OPT_DIM,OPT_DIM>::Identity();
    }
    (JtJ).llt().solveInPlace(dDeltaP);

    //this is a hack for now, but it should take care of large deltas
    if(dDeltaP.norm() > 100 ){
        JtJ += Eigen::Matrix<double,OPT_DIM,OPT_DIM>::Identity();
        dDeltaP = J.transpose()*omega*error;
        (JtJ).llt().solveInPlace(dDeltaP);
    }

    if(g_verbose){
        dout("Gauss newton delta: [" << dDeltaP.transpose().format(CleanFmt) << "]");
    }


    if(std::isfinite(dDeltaP.norm()) == false){
        dout(problem.plan_id_ << ":Deltas are NAN. Dump => J:" << J.format(CleanFmt) << std::endl << "b:" << error.transpose().format(CleanFmt) << std::endl);
        problem.planner_error_ = eDeltaNan;
        return false;
    }

    //if no damping is required, just pick the result of the gauss newton
    if(g_disable_damping == true){
        problem.UpdateOptParams(problem.current_solution_.optimization_params_.head(OPT_DIM) + dDeltaP);
        problem.boundary_solver_->Solve(&problem.boundary_problem_);
        SimulateTrajectory( problem.current_solution_.sample_,problem);
        problem.current_solution_.norm_ = _CalculateErrorNorm(problem,_CalculateSampleError(problem,problem.current_solution_.min_trajectory_time_));
        problem.local_solutions_.push_back(problem.current_solution_);
        problem.best_solution_ = &problem.local_solutions_.back();
        dout("Iteration with no damping finished with norm " << problem.current_solution_.norm_ << " and opts "<< problem.current_solution_.optimization_params_.transpose().format(CleanFmt));
        return true;
    }

    //initialize the parameters here, in case non of the
    //dampings are any good
    double damping = 0;
    //damp the gauss newton response

    Eigen::Vector6d pDampingStates[DAMPING_STEPS];
    Eigen::VectorXd pDampingErrors[DAMPING_STEPS];
    std::vector<std::shared_ptr<LocalProblem> > vCubicProblems;
    std::vector<std::shared_ptr<ApplyCommandsThreadFunctor> >vFunctors;
    vCubicProblems.resize(DAMPING_STEPS);
    vFunctors.resize(DAMPING_STEPS);
    double dampings[DAMPING_STEPS];
    damping = 1.0;


    LocalProblemSolution damped_solution;
    damped_solution.norm_ = DBL_MAX;
    if(std::isfinite(dDeltaP[0]) ){
        //create the damped problems and run the thread queue to sample them
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            dampings[ii] = damping;
            Eigen::VectorXd delta = dDeltaP *damping;
            vCubicProblems[ii] = std::make_shared<LocalProblem>(problem);
            vCubicProblems[ii]->UpdateOptParams(vCubicProblems[ii]->current_solution_.optimization_params_.head(OPT_DIM)+delta);
            vFunctors[ii] = std::make_shared<ApplyCommandsThreadFunctor>(this,
                                                                         *vCubicProblems[ii],
                                                                         ii,
                                                                         pDampingStates[ii],
                                                                         pDampingErrors[ii],
                                                                         vCubicProblems[ii]->current_solution_.sample_,
                                                                         true);
            thread_pool_.enqueue(*vFunctors[ii]);
            damping/= DAMPING_DIVISOR;
        }
        while(thread_pool_.busy_threads() > (thread_pool_.num_threads())){
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }


        if(g_verbose){
            std::cout << "Damping norms are: [";
        }
        //pick out the best damping by comparing the norms
        for(int ii = 0 ; ii < DAMPING_STEPS ; ii++) {
            double norm = 0;
            dout_cond("Final state is " << pDampingStates[ii].transpose().format(CleanFmt),g_verbose);
            norm = _CalculateErrorNorm(*vCubicProblems[ii],pDampingErrors[ii]);
            if(g_verbose){
                std::cout << " " << norm;
            }
            if(norm < damped_solution.norm_ ) {
                damped_solution = vCubicProblems[ii]->current_solution_;
                damped_solution.norm_ = norm;
                bestDamping = dampings[ii];
                nBestDampingIdx = ii;
            }
        }

        if(g_verbose){
            std::cout << "]" << std::endl;
        }

    }else{
        Eigen::IOFormat CleanFmt(2, 0, ", ", "\n", "[", "]");
        dout(problem.plan_id_ << ":Deltas ar NAN. Dump => J:" << J.format(CleanFmt) << std::endl);
        problem.planner_error_ = eDeltaNan;
        return false;
    }

    LocalProblemSolution newSolution;

    if(coordinate_descent.norm_ > problem.current_solution_.norm_ && g_monotonic_cost){
        problem.in_local_minimum_ = true;
        dout(problem.plan_id_ << ":In local minimum with Norm = " << problem.current_solution_.norm_ );
    }else if( damped_solution.norm_ > problem.current_solution_.norm_ && g_monotonic_cost) {
        problem.in_local_minimum_ = true;
        dout(problem.plan_id_ << ":Accepted coordinate descent with norm = " << problem.current_solution_.norm_ ) ;
    }else{
        newSolution = damped_solution;
        dout( problem.plan_id_ << ":New norm from damped gauss newton = " << newSolution.norm_ << " with damping = " << bestDamping << " best damped traj error is " << pDampingErrors[nBestDampingIdx].transpose().format(CleanFmt));
    }


    //update the best solution if necessary
    if(problem.in_local_minimum_ == false){
        if(newSolution.norm_ < problem.current_solution_.norm_){
            //add this solution to the list
            problem.local_solutions_.push_back(newSolution);
            problem.best_solution_ = &problem.local_solutions_.back();
        }
        problem.current_solution_ = newSolution;
    }

    problem.UpdateOptParams(problem.current_solution_.optimization_params_.head(OPT_DIM));
    problem.boundary_solver_->Solve(&problem.boundary_problem_);

    problem.planner_error_ = eSuccess;
    return true;
}
