#pragma once

#include <CarPlanner/utils/cvar_helpers.h>
#include <CarPlanner/utils/vector.h>
#include <CarPlanner/solvers/local_planner.h>
#include <CarPlanner/vehicle_parameters.h>
#include <CarPlanner/apply_velocities_functor.h>
#include <CarPlanner/ninjacar.h>

/// Moved from bullet_vehicle.h --- these look like control parameters.
#define MIN_CONTROL_DELAY 0.0
#define MAX_CONTROL_DELAY 0.3

typedef std::list<ControlPlan*> PlanPtrList;

///////////////////////////////////////////////////////////////////////////////
class CarController
{
public:
    CarController();

    void Init(std::vector<MotionSample> &segment_samples,
              LocalPlanner *planner,
              std::shared_ptr<carplanner::NinjaCar<Vehicle> > vehicle,
              double dt) ;
    void Reset();

    void current_commands(const double time, ControlCommand &command);
    VehicleState current_pose();
    /// SetCurrentPoseFromCarModel doesn't appear to be used anywhere.
    void SetCurrentPoseFromCarModel(std::shared_ptr<carplanner::NinjaCar<Vehicle> > vehicle, int world_id);
    void SetCurrentPose(VehicleState pose, CommandList* command_list = NULL);
    void GetCurrentCommands(const double time,
                            ControlCommand& command,
                            Eigen::Vector3d& target_velocity,
                            Sophus::SE3d &dT_target);
    float* max_control_plan_time_ptr() { return &max_control_plan_time_; }
    float* lookahead_time_ptr() { return &lookahead_time_; }
    double GetLastPlanStartTime();
    bool PlanControl(double plan_start_time, ControlPlan*& plan_out);
    static double AdjustStartingSample(const std::vector<MotionSample>& segment_samples,
                                       VehicleState& state,
                                       int& segment_index,
                                       int& sample_index,
                                       int lower_limit = 100,
                                       int upper_limit = 100);
    static void PrepareLookaheadTrajectory(const std::vector<MotionSample>& segment_samples,
                                           ControlPlan *plan,
                                           VelocityProfile &trajectoryProfile,
                                           MotionSample &trajectory_sample,
                                           const double dLookaheadTime);

private:
    bool _SampleControlPlan(ControlPlan* plan,LocalProblem& problem);
    bool _SolveControlPlan(const ControlPlan* plan, LocalProblem& problem, const MotionSample &trajectory);

    void _GetCurrentPlanIndex(double currentTime, PlanPtrList::iterator& planIndex, int& sample_index, double& interpolationAmount);



    int last_current_plan_index_;
    VehicleState current_state_;
    std::shared_ptr<carplanner::NinjaCar<Vehicle>> vehicle_;
    LocalPlanner* planner_;
    ControlCommand last_command_;

    double start_time_;
    double timestep_;
    bool whether_started_;
    bool whether_stopping_;
    bool state_updated_;
    bool whether_first_pose_;

    float& max_control_plan_time_;
    float& lookahead_time_;

    std::thread* control_planner_thread_;

    PlanPtrList control_plans_list_;
    std::vector<MotionSample> segment_samples_;
    CommandList current_commands_list_;

    MotionSample motion_sample_2donly_;

    std::mutex plan_mutex_;
    std::mutex state_mutex_;

    Eigen::Vector5d last_delta_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW


};


