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
#include <CarPlanner/bullet/bullet_car_model.h>

#define CAR_GRAVITY_COMPENSATION_COEFFICIENT 1.0
#define CAR_STEERING_COMPENSATION_COEFFICIENT 0

#define ROLL_TORQUE_CONTROL_ENABLED 0
#define PITCH_TORQUE_CONTROL_ENABLED 1
#define YAW_TORQUE_CONTROL_ENABLED 0


///////////////////////////////////////////////////////////////////////////////
struct ControlSample
{
public:
    double m_dSpeed;
    double steering_cmd_;
    double m_Dt;
};

struct MotionSample
{
    std::vector<VehicleState> m_vStates;
    std::vector<ControlCommand> m_vCommands;

    double MaxCommandCurvature() const
    {
        double dMax = 0;//DBL_MIN;
        for(size_t ii = 0 ; ii < m_vCommands.size() ; ii++){
            //dMax = std::max(dMax,m_vCommands[ii].curvature_);
            dMax += m_vCommands[ii].curvature_;
        }
        return dMax;
    }

    CommandList GetDelayedCommandList(const double& delay, const int& nStartIndex)
    {
        CommandList prevCommands;
        double totalDelay = delay;
        for(int kk = nStartIndex ; kk >= 0 && totalDelay > 0 ; kk--) {
            prevCommands.push_back(m_vCommands[kk]);
            totalDelay -= m_vCommands[kk].m_dT;
        }
        return prevCommands;
    }

    const VehicleState& GetLastPose() const { return m_vStates.back(); }

    std::vector<Sophus::SE3d> GetMotionSample() const
    {
        std::vector<Sophus::SE3d> vPoses;
        vPoses.reserve(m_vStates.size());
        for(const VehicleState& state : m_vStates){
            vPoses.push_back(state.t_wv_);
        }
        return vPoses;
    }

    double GetBadnessCost() const
    {
        double cost = 0;
        if(m_vCommands.size() != 0){
            const ControlCommand* pPrevCommand = &m_vCommands.front();
            for(size_t ii = 1; ii < m_vStates.size() ; ii++){
                //const VehicleState& state = m_vStates[ii];
                const ControlCommand& command = m_vCommands[ii];
                //cost = std::max(state.vel_w_dot_.norm() * state.omega_w_dot_.norm(),cost);
                //cost += fabs(state.vel_w_dot_.norm() * state.omega_w_dot_[2]) - fabs(m_vCommands[ii].curvature_);
                cost += fabs(command.m_dPhi - pPrevCommand->m_dPhi);
                pPrevCommand = &m_vCommands[ii];
                //cost += fabs(state.steering_cmd_);
            }
            cost /= GetDistance();
        }
        return cost;
    }

    double GetDistance() const
    {
        double dist = 0;
        if(m_vStates.empty() == false){
            Eigen::Vector3d lastPos = m_vStates[0].t_wv_.translation();
            for(const VehicleState& state : m_vStates){
                dist += (state.t_wv_.translation()-lastPos).norm();
                lastPos = state.t_wv_.translation();
            }
        }
        return dist;
    }

    ////////////////////////////////////////////////////////////////
    static bool FixSampleIndexOverflow(const std::vector<MotionSample>& segmentSamples, int& segmentIndex, int& sampleIndex, bool loop = true)
    {
        bool overFlow = false;
        bool underFlow = false;

        //if this index is beyond the bounds, we move to the next segment
        while(sampleIndex >= (int)segmentSamples[segmentIndex].m_vStates.size()) {
            sampleIndex -= (int)segmentSamples[segmentIndex].m_vStates.size();
            segmentIndex++;

            //if we have reached the end of the segments, we can loop back
            if(segmentIndex >= (int)segmentSamples.size()) {
                if(loop){
                    //loop around
                    segmentIndex = 0;
                }else{
                    //do not loop around
                    segmentIndex = segmentSamples.size()-1;
                    sampleIndex = (int)segmentSamples[segmentIndex].m_vStates.size();
                }
            }
            overFlow = true;
        }

        while(sampleIndex < 0) {
            segmentIndex--;

            //if we have reached the beginning of the segments, we can loop back
            if(segmentIndex < 0) {
                segmentIndex = segmentSamples.size()-1;
            }

            sampleIndex += (int)segmentSamples[segmentIndex].m_vStates.size();
            underFlow = true;
        }

        return overFlow || underFlow;
    }

    void Clear(){
        m_vStates.clear();
        m_vCommands.clear();
    }
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ControlPlan
{
public:

    double m_dStartTime;
    double m_dEndTime;
    double m_dNorm;
    MotionSample m_Sample;

    Sophus::SE3d m_dStartPose;
    Sophus::SE3d m_dEndPose;

    int m_nStartSegmentIndex;   //the segment at which this plan starts
    int m_nStartSampleIndex; //the sample in the segment at which this control plan starts

    int m_nEndSegmentIndex;   //the segment at which this plan ends
    int m_nEndSampleIndex; //the sample in the segment at which this control plan ends
    int m_nPlanId;


    VehicleState m_StartState;
    Eigen::Vector3d m_dStartTorques;
    VehicleState m_GoalState;

    void Clear() {
        m_Sample.Clear();
    }

    ~ControlPlan(){
        //dout("Deleting control plan.");
    }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ApplyVelocitesFunctor5d
{
public:
    ApplyVelocitesFunctor5d(std::shared_ptr<carplanner::NinjaCar<Vehicle,Controller>> vehicle,
                            Eigen::Vector3d init_torques,
                            CommandList* previous_commands = NULL);

    VehicleState ApplyVelocities(const VehicleState &start_state,
                                 MotionSample& sample,
                                 int index = 0,
                                 bool no_compensation = false);

    void ApplyVelocities(const VehicleState &startingState,
                         std::vector<ControlCommand> &m_vCommands,
                         std::vector<VehicleState>& vStatesOut,
                         const int nStartIndex,
                         const int nEndIndex,
                         const int nIndex,
                         const bool noCompensation = false,
                         const CommandList *pPreviousCommands = NULL);


    double max_wheel_torque(const VehicleState& state, const int nIndex);
    double gravity_compensation(int nIndex);
    double steering_compensation(VehicleState& state, double phi, double curvature, int nIndex);
    double friction_compensation(int nIndex, double dt);

    std::shared_ptr<carplanner::NinjaCar<Vehicle,Controller>> vehicle(){ return vehicle_; }
    const std::shared_ptr<carplanner::NinjaCar<Vehicle,Controller>> vehicle() const{ return vehicle_; }
    CommandList& previous_command() { return previous_commands_; }
    void set_previous_commands(const CommandList& list) { previous_commands_ = list;}
    void reset_previous_commands() { return previous_commands_.clear(); }
    bool set_no_delay(bool no_delay){ return (no_delay_ = no_delay); }
private:
    std::shared_ptr<carplanner::NinjaCar<Vehicle,Controller>> vehicle_;
    Eigen::Vector3d init_torques_;
    CommandList previous_commands_;
    bool no_delay_;
};

