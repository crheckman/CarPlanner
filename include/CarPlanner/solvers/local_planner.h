#pragma once

#include <sophus/se3.hpp>
#include <sophus/se2.hpp>
#include <CarPlanner/ninjacar.h>
#include <CarPlanner/boundary_solver.h>
#include <CarPlanner/apply_velocities_functor.h>
#include <CarPlanner/utils/thread_pool.h>
#include <CarPlanner/solvers/bezier_boundary_solver.h>

#define XYZ_WEIGHT 2
#define THETA_WEIGHT 0.5
#define VEL_WEIGHT_TRAJ 0.5
#define VEL_WEIGHT_POINT 1.0
#define TIME_WEIGHT 0.05
#define CURV_WEIGHT 0.001
#define BADNESS_WEIGHT 5e-8;
#define DAMPING_STEPS 8
#define DAMPING_DIVISOR 1.3

#define POINT_COST_ERROR_TERMS 5
#define TRAJ_EXTRA_ERROR_TERMS 2
#define TRAJ_UNIT_ERROR_TERMS 5

#define OPT_ACCEL_DIM 3
#define OPT_AGGR_DIM 4
#define OPT_DIM 4

#define PLANNER_NUM_THREADS 8

enum PlannerError
{
    eSuccess,
    eJacobianColumnNan,
    eJacobianColumnZero,
    eDeltaNan,
};

struct VelocityProfileNode
{
    VelocityProfileNode(const double& dDistanceRatio, const double& dVel) : m_dDistanceRatio(dDistanceRatio),vel_w_dot_el(dVel){}
    double m_dDistanceRatio;
    double vel_w_dot_el;
};

struct AccelerationProfileNode
{
    AccelerationProfileNode(const double& end_time, const double& accel, const double& end_dist,
                            const double& vel_dot_start, const double& vel_dot_end) :
        end_time_(end_time),
        accel_(accel),
        end_dist_(end_dist),
        vel_dot_start_(vel_dot_start),
        vel_dot_end_(vel_dot_end){}
    double end_time_;
    double accel_;
    double end_dist_;
    double vel_dot_start_;
    double vel_dot_end_;
};

typedef std::vector<VelocityProfileNode > VelocityProfile;
typedef std::vector<AccelerationProfileNode > AccelerationProfile;

struct LocalProblemSolution
{
    LocalProblemSolution() {}
    LocalProblemSolution(const MotionSample& sample,
                         const Eigen::Vector5d& optimization_params,
                         const double min_trajectory_time,
                         const double norm):
        optimization_params_(optimization_params),
        sample_(sample),
        min_trajectory_time_(min_trajectory_time),
        norm_(norm)
    {

    }
    //BezierBoundaryProblem m_Solution;
    Eigen::Vector5d optimization_params_;
    MotionSample sample_;
    double min_trajectory_time_;
    double norm_;
};

enum LocalProblemCostMode
{
    eCostPoint,
    eCostTrajectory
};

struct LocalProblem
{
    void Reset()
    {
        torque_start_time_ = -1;
        coefs_ = Eigen::Vector4d::Zero();
        start_torques_ = Eigen::Vector3d::Zero();
        inertial_control_active_ = false;
        best_solution_ = nullptr;
        local_solutions_.clear();;
    }

    LocalProblem()
    {
        Reset();
    }

    LocalProblem(ApplyVelocitesFunctor5d* m_pf, const VehicleState& startState, const VehicleState& goalState, const double& dt) :segment_time_(-1), start_time_(-1.0),
        timestep_(dt),functor_(m_pf)
    {
        Reset();
        start_state_ = startState;
        goal_state_ = goalState;
    }

    int plan_id_;

    VehicleState start_state_;
    VehicleState goal_state_;

    Eigen::Vector6d start_pose_;   //< Starting 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature,v]'
    Eigen::Vector6d m_goal_pose_;    //< Goal 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature,v]'
    Sophus::SO3d t_inv_;
    Sophus::SE3d t_3d_inv_;
    Sophus::SE3d t_3d_;

    //Eigen::VectorCubic2D m_dCubic;
    double segment_time_;
    double max_segment_time_;                   //< This is to stop runaway simulation in case the gauss newton delta destroys the solution

    VelocityProfile velocity_profile_;              //< Velocity profile for this trajectory
    AccelerationProfile accel_profile_;        //< Acceleration profile for this trajectory

    double start_time_;

    //Eigen::Vector5d optimization_params_;               //< Optimization parameters, which are parametrized as [x,y,t,a]
    Eigen::Vector5d initial_optimization_params_;
    Eigen::Vector6d transformed_goal_;
    double timestep_;                                //< The dt used in the functor to push the simulation forward

    ApplyVelocitesFunctor5d* functor_;        //< The functor which is responsible for simulating the car dynamics

    //optimization related properties
    BezierBoundaryProblem boundary_problem_;           //< The boundary problem structure, describing the 2D boundary problem
    BoundarySolver* boundary_solver_;          //< Pointer to the boundary value solver which will be used to solve the 2D problem
    //double current_norm_;                      //< The current norm of the optimization problem
    //MotionSample* m_pCurrentMotionSample;       //< Pointer to the current motion sample (which represents the current 3D trajectory)
    bool in_local_minimum_;                     //< Boolean which indicates if we're in a local minimum, which would indicate that the optimization is finished
    PlannerError planner_error_;

    LocalProblemCostMode cost_mode_;
    MotionSample trajectory_sample_;
    Eigen::Vector6dAlignedVec transformed_trajectory_;
    //double min_trajectory_time_;

    //double m_dDistanceDelta;
    Eigen::Vector4d coefs_;
    bool inertial_control_active_;
    double torque_start_time_;
    double torque_duration_;
    Eigen::Vector3d start_torques_;

    void UpdateOptParams(const Eigen::VectorXd& optimization_params)
    {
        current_solution_.optimization_params_.head(OPT_DIM) = optimization_params;
        boundary_problem_.m_goal_pose_.head(3) = optimization_params.head(3);
        if(OPT_DIM > OPT_AGGR_DIM){
            //dout("Setting opt params to " << optimization_params.transpose());
            boundary_problem_.aggressiveness_ = current_solution_.optimization_params_[OPT_AGGR_DIM];
        }
    }

    std::list<LocalProblemSolution> local_solutions_;
    LocalProblemSolution* best_solution_;
    LocalProblemSolution current_solution_;
};

inline Eigen::VectorXd GetPointLineError(const Eigen::Vector6d& line1, const Eigen::Vector6d& line2, const Eigen::Vector6d& point, double &interpolation_factor);

class LocalPlanner
{
public:
    LocalPlanner();
    /// Initializes the LocalProblem structu which is passed in using the given parameters
    bool InitializeLocalProblem(LocalProblem& problem,  //< The local problem struct which will be fixed
                                const double start_time, //< Starting time of the problem, this is used to parametrized the generated command laws
                                const VelocityProfile* velocity_profile  = NULL,
                                LocalProblemCostMode eCostMode = eCostPoint);
    /// Given a LocalProblem struct which contains a velocity profile, this function obtains the corresponding
    /// acceleration profile based
    void _GetAccelerationProfile(LocalProblem& problem) const;
    /// Iterate the given problem using gauss-newton
    bool Iterate(LocalProblem &problem);
    /// Samples the 2D control law that is generated by solving the boundary value problem
    void SamplePath(const LocalProblem &problem,
                    Eigen::Vector3dAlignedVec &samples,
                    bool best_solution = false);
    /// Given the local problem struct, will simulate the vehicle physics and produce a motion sample
    Eigen::Vector6d SimulateTrajectory(MotionSample& sample,     //< The motion sample which will be filled by the function
                                       LocalProblem& problem,    //< The Local Problem structure which will define the trajectory
                                       const int nIndex = 0,      //< The index of the world to run the simulation in (this is to do with the thread)
                                       const bool &best_solution = false);
    /// Samples the acceleration and curvature of the current control law
    void SampleAcceleration(std::vector<ControlCommand>& command_vector,
                            LocalProblem &problem) const;
    void CalculateTorqueCoefficients(LocalProblem &problem,
                                     MotionSample *motion_sample);
    /// Calculates the error for the current trajectory. The error is parametrized as [x,y,t,v]
    Eigen::VectorXd _CalculateSampleError(LocalProblem& problem, double& min_traj_time) const { return _CalculateSampleError(problem.current_solution_.sample_,problem,min_traj_time); }
    Eigen::VectorXd _CalculateSampleError(const MotionSample &sample, LocalProblem &problem, double &min_traj_time) const;
    Eigen::VectorXd _GetWeightVector(const LocalProblem& problem);
    double _CalculateErrorNorm(const LocalProblem &problem, const Eigen::VectorXd& dError);
    static int GetNumWorldsRequired(const int num_optimization_params) { return num_optimization_params*2+2;}
private:
    /// Calculates the jacobian of the trajectory at the current point in the trajectory
    bool _CalculateJacobian(LocalProblem &problem,          //< The problem struct defining the current trajectory and goals
                              Eigen::VectorXd& dCurrentErrorVec,          //< This is the current error vector
                              LocalProblemSolution& coordinate_descent,
                              Eigen::MatrixXd &J                    //< Output: The jacobian matrix
                              );
    /// Internal function that iterates the gauss-newton optimization step
    bool _IterateGaussNewton( LocalProblem& problem );

    /// Calculates the distance travelled, given the time passed
    double _DistanceTraveled( const double& t,const AccelerationProfile& profile ) const;

    /// Transforms the goal pose so that it is on the 2D manifold specified by the problem struct
    Eigen::Vector6d _TransformGoalPose(const Eigen::Vector6d &goal_pose, const LocalProblem& problem) const;
    /// Returns the trajectory error given the trajectory and a transformed trajectory and end pose
    Eigen::VectorXd _GetTrajectoryError(const MotionSample& sample, const Eigen::Vector6dAlignedVec& transformed_poses, const Eigen::Vector6d& end_pose, double &min_time) const;
    /// Transforms a vehicle state so that it is on the 2D manifold specified by the problem struct
    Eigen::Vector6d _Transform3dGoalPose(const VehicleState& state, const LocalProblem &problem) const;

    ThreadPool thread_pool_;

    double& eps_;                                              //< The epsilon used in the calculation of the finite difference jacobian


    BezierBoundarySolver boundary_solver_;                      //< The boundary value problem solver

    Eigen::MatrixXd& point_weight_;                                       //< The matrix which holds the weighted Gauss-Newton weights
    Eigen::MatrixXd& traj_weight_;
    int plan_counter_;
};
