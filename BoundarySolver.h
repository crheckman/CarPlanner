#ifndef BOUNDARYSOLVER_H
#define BOUNDARYSOLVER_H

#include "CarPlannerCommon.h"

/// A structure containing the 2D boundary value problem inputs and outputs. This is passed to the boundary solver
/// class in order to obtain a valid control law, which is also written to this structure.
struct BoundaryProblem
{
    enum BoundaryProblemType
    {
        eBoundary_Point = 1,
        eBoundary_Trajectory = 2
    };

    BoundaryProblem()
    { }

    virtual ~BoundaryProblem() {}

//    BoundaryProblem(const BoundaryProblem& problem) : m_dStartPose(problem.m_dStartPose),m_dGoalPose(problem.m_dGoalPose),
//        m_dMaxCurvature(problem.m_dMaxCurvature), m_nDiscretization(problem.m_nDiscretization), m_vCurvatures(problem.m_vCurvatures),
//        m_vPts(problem.m_vPts), m_dDistance(problem.m_dDistance)
//    {}

    //inputs into the solver
    Eigen::Vector4d m_dStartPose;   //< Starting 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature]'
    Eigen::Vector4d m_dGoalPose;    //< Goal 2D pose for the boundary value solver, which is parametrized as [x,y,theta,curvature]'

    Eigen::Vector4dAlignedVec m_vTrajectory;
    double m_dMaxCurvature;         //< Maximum curvature desired from the trajectory. This could correspond to the maximum turning capability of the vehicle
    int m_nDiscretization;          //< The number of steps required along the trajectory

    //outputs from the solver
    std::vector<double> m_vCurvatures;      //< Output: Curvature law generated by the solver. The length is equal to m_nDiscretization
    Eigen::Vector2dAlignedVec m_vPts;    //< Output: The 2d points of the control law. The length is equal to m_nDiscretization
    double m_dDistance;                     //< Output: The total distance of the control law generated. This is in meters
    BoundaryProblemType m_eType;
};

class BoundarySolver
{
public:
    BoundarySolver() {}
    virtual ~BoundarySolver() {}
    /// Solves the 2D boundary problem with the given problem description struct
    virtual void Solve(BoundaryProblem* pProblem) = 0;
    /// Get the curvature at a particular distance along the curve
    virtual double GetCurvature(const BoundaryProblem *pProblem, double dist) = 0;
};

#endif // BOUNDARYSOLVER_H
