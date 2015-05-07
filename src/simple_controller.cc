#include "CarPlanner/Controller/pid.h"
#include "CarPlanner/Controller/common.h"

// Initialize the control law.
void CarController::Init(const BulletCarModel *vehicle,
                         const Eigen::VectorXd *currentPose,
                         const Eigen::VectorXd *end_pose)

