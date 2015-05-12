#pragma once

#include <vector>
#include <cmath>
#include <float.h>
#include <thread>
#include <queue>

#include <Eigen/LU>
#include <Eigen/Core>

#include <CVars/CVar.h>

#include <CarPlanner/ninjacar.h>
#include <CarPlanner/vehicle_state.h>
#include <CarPlanner/vehicle_parameters.h>

#define CAR_GRAVITY_COMPENSATION_COEFFICIENT 1.0
#define CAR_STEERING_COMPENSATION_COEFFICIENT 0

#define ROLL_TORQUE_CONTROL_ENABLED 0
#define PITCH_TORQUE_CONTROL_ENABLED 1
#define YAW_TORQUE_CONTROL_ENABLED 0

namespace carplanner {
///////////////////////////////////////////////////////////////////////////////
struct ControlSample
{
public:
    double speed_;
    double steering_cmd_;
    double timestep_;
};

struct MotionSample
{
    std::vector<VehicleState> states_vector_;
    std::vector<ControlCommand> commands_vector_;

    double MaxCommandCurvature() const
    {
        double dMax = 0;//DBL_MIN;
        for(size_t ii = 0 ; ii < commands_vector_.size() ; ii++){
            dMax += commands_vector_[ii].curvature_;
        }
        return dMax;
    }

    CommandList GetDelayedCommandList(const double& delay, const int& start_index)
    {
        CommandList precommand_vector;
        double total_delay = delay;
        for(int kk = start_index ; kk >= 0 && total_delay > 0 ; kk--) {
            precommand_vector.push_back(commands_vector_[kk]);
            total_delay -= commands_vector_[kk].timestep_;
        }
        return precommand_vector;
    }

    const VehicleState& GetLastPose() const { return states_vector_.back(); }

    std::vector<Sophus::SE3d> GetMotionSample() const
    {
        std::vector<Sophus::SE3d> vPoses;
        vPoses.reserve(states_vector_.size());
        for(const VehicleState& state : states_vector_){
            vPoses.push_back(state.t_wv_);
        }
        return vPoses;
    }

    double GetBadnessCost() const
    {
        double cost = 0;
        if(commands_vector_.size() != 0){
            const ControlCommand* pPrevCommand = &commands_vector_.front();
            for(size_t ii = 1; ii < states_vector_.size() ; ii++){
                //const VehicleState& state = states_vector_[ii];
                const ControlCommand& command = commands_vector_[ii];
                //cost = std::max(state.vel_w_dot_.norm() * state.omega_w_dot_.norm(),cost);
                //cost += fabs(state.vel_w_dot_.norm() * state.omega_w_dot_[2]) - fabs(commands_vector_[ii].curvature_);
                cost += fabs(command.phi_ - pPrevCommand->phi_);
                pPrevCommand = &commands_vector_[ii];
                //cost += fabs(state.steering_cmd_);
            }
            cost /= GetDistance();
        }
        return cost;
    }

    double GetDistance() const
    {
        double dist = 0;
        if(states_vector_.empty() == false){
            Eigen::Vector3d lastPos = states_vector_[0].t_wv_.translation();
            for(const VehicleState& state : states_vector_){
                dist += (state.t_wv_.translation()-lastPos).norm();
                lastPos = state.t_wv_.translation();
            }
        }
        return dist;
    }

    ////////////////////////////////////////////////////////////////
    static bool FixSampleIndexOverflow(const std::vector<MotionSample>& segment_samples, int& segment_index, int& sample_index, bool loop = true)
    {
        bool overFlow = false;
        bool underFlow = false;

        //if this index is beyond the bounds, we move to the next segment
        while(sample_index >= (int)segment_samples[segment_index].states_vector_.size()) {
            sample_index -= (int)segment_samples[segment_index].states_vector_.size();
            segment_index++;

            //if we have reached the end of the segments, we can loop back
            if(segment_index >= (int)segment_samples.size()) {
                if(loop){
                    //loop around
                    segment_index = 0;
                }else{
                    //do not loop around
                    segment_index = segment_samples.size()-1;
                    sample_index = (int)segment_samples[segment_index].states_vector_.size();
                }
            }
            overFlow = true;
        }

        while(sample_index < 0) {
            segment_index--;

            //if we have reached the beginning of the segments, we can loop back
            if(segment_index < 0) {
                segment_index = segment_samples.size()-1;
            }

            sample_index += (int)segment_samples[segment_index].states_vector_.size();
            underFlow = true;
        }

        return overFlow || underFlow;
    }

    void Clear(){
        states_vector_.clear();
        commands_vector_.clear();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ControlPlan
{
public:

    double start_time_;
    double end_time_;
    double norm_;
    MotionSample sample_;

    Sophus::SE3d start_pose_;
    Sophus::SE3d end_pose_;

    int start_segment_index_;   //the segment at which this plan starts
    int start_sample_index_; //the sample in the segment at which this control plan starts

    int ending_segment_index_;   //the segment at which this plan ends
    int ending_sample_index_; //the sample in the segment at which this control plan ends
    int plan_id_;


    VehicleState start_state_;
    Eigen::Vector3d start_torques_;
    VehicleState goal_state_;

    void Clear() {
        sample_.Clear();
    }

    ~ControlPlan(){
        //dout("Deleting control plan.");
    }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ApplyVelocitesFunctor5d
{
public:
    ApplyVelocitesFunctor5d(std::shared_ptr<carplanner::NinjaCar> vehicle,
                            Eigen::Vector3d init_torques,
                            CommandList* previous_commands = NULL);

    VehicleState ApplyVelocities(const VehicleState &start_state,
                                 MotionSample& sample,
                                 int index = 0,
                                 bool no_compensation = false);

    void ApplyVelocities(const VehicleState &start_state,
                         std::vector<ControlCommand> &commands_vector,
                         std::vector<VehicleState>& states_out,
                         const int start_index,
                         const int nEndIndex,
                         const int nIndex,
                         const bool whether_compensation = false,
                         const CommandList *pPreviousCommands = NULL);


    double max_wheel_torque(const VehicleState& state, const int nIndex);
    double gravity_compensation(int nIndex);
    double steering_compensation(VehicleState& state, double phi, double curvature, int nIndex);
    double friction_compensation(int nIndex, double dt);

    std::shared_ptr<carplanner::NinjaCar> vehicle(){ return vehicle_; }
    const std::shared_ptr<carplanner::NinjaCar> vehicle() const{ return vehicle_; }
    CommandList& previous_command() { return previous_commands_; }
    void set_previous_commands(const CommandList& list) { previous_commands_ = list;}
    void reset_previous_commands() { return previous_commands_.clear(); }
    bool set_no_delay(bool no_delay){ return (no_delay_ = no_delay); }
private:
    std::shared_ptr<carplanner::NinjaCar> vehicle_;
    Eigen::Vector3d init_torques_;
    CommandList previous_commands_;
    bool no_delay_;
};

}
