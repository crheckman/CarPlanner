#pragma once

#include <CarPlanner/ninjacar.h>
#include <CarPlanner/vehicle_state.h>
#include <CarPlanner/vehicle_parameters.h>
#include <CarPlanner/control_command.h>

#include <CarPlanner/solvers/local_planner.h>
#include <CarPlanner/utils/optim.h>
#include <CarPlanner/utils/thread_pool.h>

#define NORM_NOT_INITIALIZED -1
#define REGRESSOR_NUM_THREADS 8
#define REGRESSOR_NUM_WORLDS 11

class CarRegressor
{
public:
  CarRegressor();
  void Init(double dEpsilon,std::fstream* pLogFile = NULL);
  void Regress(ApplyVelocitesFunctor5d& f,
               MotionSample &sample,
               const std::vector<RegressionParameter>& params,
               std::vector<RegressionParameter>& newParams);

  void ApplyParameters(ApplyVelocitesFunctor5d f,
                       MotionSample &plan,
                       const std::vector<RegressionParameter>& params,
                       const int index,
                       Eigen::Vector7d& errorOut,
                       int start_index,
                       int nEndIndex,
                       std::vector<VehicleState> *pStatesOut = NULL,
                       CommandList *pPreviousCommands = NULL);

  void CalculateJacobian(ApplyVelocitesFunctor5d f, MotionSample &sample,
                         const std::vector<RegressionParameter>& params, std::vector<RegressionParameter>& vBestParams,
                         double& dBestNorm, int &nBestDimension, Eigen::MatrixXd &JtJ, Eigen::VectorXd &Jb);

  double CalculateParamNorms(ApplyVelocitesFunctor5d f,
                             MotionSample& plan,
                             const std::vector<RegressionParameter>& params,
                             std::vector<MotionSample> *motion_samples = NULL,
                             std::vector<int> *motion_sampleIndices = NULL);

  Eigen::MatrixXd FiniteDiffFunctor(ApplyVelocitesFunctor5d f,
                                    MotionSample& plan,
                                    const std::vector<RegressionParameter>& params,
                                    Eigen::VectorXd& vBestParams,
                                    Eigen::Vector7d& dBaseError,
                                    double& dBestNorm,
                                    int& nBestDimension,
                                    int start_index,
                                    int nEndIndex);



private:
  void _RefreshIndices(MotionSample &sample, ApplyVelocitesFunctor5d &f);
  void _OptimizeParametersGN(ApplyVelocitesFunctor5d& f,
                             MotionSample& plan,
                             const std::vector<RegressionParameter>& params_in,
                             std::vector<RegressionParameter>& params_out,
                             double& new_norm);

  ThreadPool thread_pool_;
  double epsilon_;
  double current_norm_;
  OptimizationTask current_task_;
  bool failed_;
  int segment_length_;
  int start_index_;
  std::fstream* logfile_;
  Eigen::MatrixXd omega_w_dot_;
  Eigen::MatrixXd prior_;
  std::vector<std::pair<int,int> > segment_indices_;
};
