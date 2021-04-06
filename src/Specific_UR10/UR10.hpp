#ifndef UR10_HPP
#define UR10_HPP

#include "genericLogic.hpp"

typedef enum
{
	ARM1,
	ARM2
}currentRobotArm;

typedef enum
{
	NO_CONTROLLER,
	SPEED_CONTROLLER,
	POSITION_CONTROLLER,
	JOINT_CONTROLLER	
}currentRobotController;

void stopCurrentController(currentRobotArm armToStop);
void startCurrentController(currentRobotArm armToStart, currentRobotController controllerToStart);

//void jointMoveArm(currentRobotArm armToMove, TBD)
void cartesianPositionMoveArm(currentRobotArm armToMove, pose armTargetPose);
void cartesianVelocityMoveArm(currentRobotArm armToMove, pose armSpeedOrder); //Type abuse. 

#endif //UR10_HPP
