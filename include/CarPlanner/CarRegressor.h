#pragma once

#include <CarPlanner/LocalPlanner.h>
#include <CarPlanner/regression_parameter.h>
#include <CarPlanner/ninjacar.h>
#include <CarPlanner/vehicle_state.h>
#include <CarPlanner/control_command.h>

#include <CarPlanner/utils/optim.h>
#include <CarPlanner/utils/thread_pool.h>

#define NORM_NOT_INITIALIZED -1
#define REGRESSOR_NUM_THREADS 8
#define REGRESSOR_NUM_WORLDS 11

namespace CarPlanner {

class CarRegressor
{
public:
    CarRegressor();
    void Init(double dEpsilon,std::fstream* pLogFile = NULL);
    void Regress(ApplyVelocitesFunctor5d& f, MotionSample &sample, const std::vector<RegressionParameter>& params, std::vector<RegressionParameter>& newParams);

    void ApplyParameters(ApplyVelocitesFunctor5d f,
                         MotionSample &plan,
                         const std::vector<RegressionParameter>& params,
                         const int index,
                         Eigen::Vector7d& errorOut,
                         int nStartIndex,
                         int nEndIndex,
                         std::vector<VehicleState> *pStatesOut = NULL,
                         CommandList *pPreviousCommands = NULL);

    void CalculateJacobian(ApplyVelocitesFunctor5d f, MotionSample &sample,
                            const std::vector<RegressionParameter>& params, std::vector<RegressionParameter>& vBestParams,
                            double& dBestNorm, int &nBestDimension, Eigen::MatrixXd &JtJ, Eigen::VectorXd &Jb);

    double CalculateParamNorms(ApplyVelocitesFunctor5d f,
                               MotionSample& plan,
                               const std::vector<RegressionParameter>& params,
                               std::vector<MotionSample> *pSamples = NULL,
                               std::vector<int> *pSampleIndices = NULL);

    Eigen::MatrixXd FiniteDiffFunctor(ApplyVelocitesFunctor5d f,
                                      MotionSample& plan,
                                      const std::vector<RegressionParameter>& params,
                                      Eigen::VectorXd& vBestParams,
                                      Eigen::Vector7d& dBaseError,
                                      double& dBestNorm,
                                      int& nBestDimension,
                                      int nStartIndex,
                                      int nEndIndex);



private:
    void _RefreshIndices(MotionSample &sample, ApplyVelocitesFunctor5d &f);
    void _OptimizeParametersGN(ApplyVelocitesFunctor5d& f, MotionSample& plan,
                               const std::vector<RegressionParameter>& dParams, std::vector<RegressionParameter>& dParamsOut,
                               double& dNewNorm);

    ThreadPool m_ThreadPool;
    double m_dEpsilon;
    double m_dCurrentNorm;
    OptimizationTask m_eCurrentTask;
    bool m_bFailed;
    int m_nSegmentLength;
    int m_nStartIndex;
    std::fstream* m_pLogFile;
    Eigen::MatrixXd m_dW;
    Eigen::MatrixXd m_dPrior;
    std::vector<std::pair<int,int> > m_vSegmentIndices;
};

} //namespace CarPlanner
