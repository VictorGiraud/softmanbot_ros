#include "specificSkilledWorkcell.hpp"
#include "genericLogic.hpp"
#include <ros/ros.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <string>

#include "PoseRPY.h"


static nodeID currentMaster = SKILLED_WORKCELL;//Switch this in different state so that 

skilledWorkcellInterface& getSkilledWorkcellInterface(void)
{
	static michelinSkilledWorkcellInterface retval;
	return retval;
}

void michelinSkilledWorkcellInterface::skilledWorkcellInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;	
}

void michelinSkilledWorkcellInterface::skilledWorkcellIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
}

void michelinSkilledWorkcellInterface::skilledWorkcellStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
}

void michelinSkilledWorkcellInterface::skilledWorkcellGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
}

void michelinSkilledWorkcellInterface::skilledWorkcellPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
}

nodeID specificSkilledWorkcellGetMaster(void)
{
	return currentMaster;
}

std::vector<pose> specificSkilledWorkcellControl(std::vector<pose> targetPoses, std::string sensorString)
{
	std::vector<pose> retval;	
	std::stringstream sensorStringStream(sensorString);
	
	pose firstArmTargetPose = targetPoses[0];
	
	//Example of how it's supposed to be used. 
	//Of course, from sensor we can extract things like robot position to perform regulation.
	//Exact syntax will vary regarding what kind of stuff your actual robot ros node need.	
	/*controllers::PoseRPY msgToSend;
	msgToSend.id = 0;
	msgToSend.position.x = firstArmTargetPose.x;
	msgToSend.position.y = firstArmTargetPose.y;
	msgToSend.position.z = firstArmTargetPose.z;
	msgToSend.orientation.roll = firstArmTargetPose.roll;
	msgToSend.orientation.pitch = firstArmTargetPose.pitch;
	msgToSend.orientation.yaw = firstArmTargetPose.yaw;		

	pubUR10.publish(msgToSend);
	retval.push_back(firstArmTargetPose);*/

	return retval;
}


void specificSkilledWorkcellRosInit(void)
{
	
}

