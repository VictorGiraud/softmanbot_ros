#include "genericLogic.hpp"
#include <ros/ros.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <sstream>

#include "deformationMSD.hpp"
#include "deformationPoints.hpp"

#include <cmath>
#include <SDL2/SDL.h>
//TODO : define Rate here

static SDL_Rect pointsToRect(Point pointToTransform);

const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

const float SpeedDist = 10;
const float maxDist = 1;

std::vector<pose> specificSkilledWorkcellControl(std::vector<pose> targetPoses, std::string sensorString)
{
	std::vector<pose> retval;	
	std::stringstream sensorStringStream(sensorString);
	
	pose firstArmTargetPose = targetPoses[0];
	pose firstArmCurrentStepTargetPose;

	DeformationMSD objectModel;
	boost::archive::text_iarchive ia(sensorStringStream);
	ia >> objectModel;
	
	Point currentRobotPosition = objectModel.dp.getPointTL();
	SDL_Rect currentRobotPositionRect = pointsToRect(currentRobotPosition);

	float deltax = firstArmTargetPose.x - currentRobotPositionRect.x;
	float deltay = firstArmTargetPose.y - currentRobotPositionRect.y;
	float dist = sqrt(pow(deltax, 2.0) + pow(deltay, 2.0));
	if(dist > maxDist)
	{
		if(dist > SpeedDist)
		{
			deltax = 0.025*SpeedDist*deltax/dist;
			deltay = 0.025*SpeedDist*deltay/dist;
		}
		else
		{		
			deltax = 0.1 * maxDist*deltax/dist;
			deltay = 0.1 * maxDist*deltay/dist;
		}
	}
	else
	{
		deltax = 0;
		deltay = 0;
	}
	
	firstArmCurrentStepTargetPose.x = deltax;
	firstArmCurrentStepTargetPose.y = deltay;

	retval.push_back(firstArmCurrentStepTargetPose);

	//Second Arm. Code duplication, this smell
	pose secondArmTargetPose = targetPoses[1];
	pose secondArmCurrentStepTargetPose;

	currentRobotPosition = objectModel.dp.getPointTR();
	currentRobotPositionRect = pointsToRect(currentRobotPosition);

	deltax = secondArmTargetPose.x - currentRobotPositionRect.x;
	deltay = secondArmTargetPose.y - currentRobotPositionRect.y;
	dist = sqrt(pow(deltax, 2.0) + pow(deltay, 2.0));
	if(dist > maxDist)
	{
		if(dist > SpeedDist)
		{
			deltax = 0.025*SpeedDist*deltax/dist;
			deltay = 0.025*SpeedDist*deltay/dist;
		}
		else
		{		
			deltax = 0.1 * maxDist*deltax/dist;
			deltay = 0.1 * maxDist*deltay/dist;
		}
	}
	else
	{
		deltax = 0;
		deltay = 0;
	}
	
	secondArmCurrentStepTargetPose.x = deltax;
	secondArmCurrentStepTargetPose.y = deltay;

	retval.push_back(secondArmCurrentStepTargetPose);

	return retval;
}

static SDL_Rect pointsToRect(Point pointToTransform)
{
	SDL_Rect retval;

	int originX = SCREEN_WIDTH/2;
	int originY = SCREEN_HEIGHT/2;

	retval.x = 10 * pointToTransform.getCoordinateX() + originX - 3;
	retval.y = - 10 * pointToTransform.getCoordinateY() + originY - 3;
	retval.w = 5;
	retval.h = 5;

	return retval;
}
