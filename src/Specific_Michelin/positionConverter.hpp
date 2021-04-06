#include "PoseRPY.h"
#include "genericLogic.hpp"
#include "Eigen/Dense"

//Driver to put everything in the same Frame

//our basis is PoseRpy(0,0,0)

Eigen::Quaternion<double> 	RPYToQuaternion(pose p);
pose 						quaternionToRPY(Eigen::Quaternion<double>);

pose EndEffector1ToUnifiedPosition(pose EndEffectorPose);
pose EndEffector2ToUnifiedPosition(pose EndEffectorPose);
pose CameraToUnifiedPosition(pose CameraPose);

pose UnifiedPositionToEndEffector1(pose unifiedPose);
pose UnifiedPositionToEndEffector2(pose unifiedPose);

