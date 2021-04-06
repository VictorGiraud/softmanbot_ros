#include "specificPerception.hpp"

#include "ros/ros.h"
#include "genericLogic.hpp"

static ros::Publisher specificSensorPub;

perceptionInterface& getPerceptionInterface(void)
{
	static michelinPerceptionInterface retval;
	return retval;
}

void michelinPerceptionInterface::perceptionInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
}

void michelinPerceptionInterface::perceptionIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
}

void michelinPerceptionInterface::perceptionStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
}

void michelinPerceptionInterface::perceptionGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
}

void michelinPerceptionInterface::perceptionPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
}	

void specificPerceptionRosInit(void)
{
	return;
}


