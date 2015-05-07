#pragma once

#include <CarPlanner/boundary_solver.h>
#include "float.h"

/// Deriving from the boundary problem class, this class only adds specific control point vectors so as to
/// facilitate the storage of these values. This will aid future versions of the solver to quickly adjust the bezier
/// curve if it has already been solved.
struct BezierBoundaryProblem : boundary_problem
{
    BezierBoundaryProblem() :
        solved_(false),
        aggressiveness_(1.2) { }

    std::vector<double> distances_;      //< Output: The distances alone
    Eigen::VectorXd x_vals_;            //< The x values of the control handles for the bezier curve
    Eigen::VectorXd y_vals_;            //< The y values of the control handles for the bezier curve
    Eigen::Vector4d solved_goal_pose_;  //< if m_bSolved is true, this vector contains the goal pose for which the bezier was solved
    Eigen::Vector4d params_;          //< The parameters that define how the control points relate
    bool solved_;                     //< Boolean value indicating whether or not the problem has already been solved.
    double          aggressiveness_;
    double          segment_length_;
};

class BezierBoundarySolver : public BoundarySolver
{
public:
    BezierBoundarySolver();
    /// Solves the problem by connecting the bezier between the start and goal poses. The problem pointer passed in must be a
    /// pointer to a BezierBoundaryProblem instance, as it contains specific values which the solve function fills.
    virtual void Solve(boundary_problem* problem);
    /// Get the curvature at a particular distance along the curve
    virtual double curvature(const boundary_problem *problem, double dist);

private:
    /// Get the polynomial coefficients for the bezier, and its first and second derivatives
    void _GetCoefs(Eigen::Vector6d& coefs, Eigen::Vector6d&  dCoefs, Eigen::Vector6d&  ddCoefs, const double &t);
    /// Samples the problem by calculating the bezier positions and curvatures
    void _Sample5thOrderBezier(BezierBoundaryProblem* problem);
    /// Given the boundary problem, this function fills in the bezier xVals and yVals to string the bezier between the given
    /// start and end goal positions
    void _Get5thOrderBezier(BezierBoundaryProblem* problem,
                           const Eigen::Vector4d& params //< Parameter vector composed of a1,b1,a2,b2
                           );

    static const int bezier_cubic_order_ = 5; //< The order of the bezier cubic that is used to generate the control law

    /// This function returns the maximum curvature of an already sample bezier curve
    double _GetMaximumCurvature(const BezierBoundaryProblem* problem);

    void _IterateCurvatureReduction(BezierBoundaryProblem* problem, Eigen::Vector4d &params);
};
