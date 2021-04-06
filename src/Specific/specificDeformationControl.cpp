#include "specificDeformationControl.hpp"

#include "genericLogic.hpp"
#include <ros/ros.h>

std::string sensorString;//This string is the serialized object detected by sensors. We can extract perception data from it

deformationControlInterface& getDeformationControlInterface(void)
{
	static michelinDeformationControlInterface retval;
	return retval;
}

void michelinDeformationControlInterface::deformationControlInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
}

void michelinDeformationControlInterface::deformationControlIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
}

void michelinDeformationControlInterface::deformationControlStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
}

void michelinDeformationControlInterface::deformationControlGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
}

void michelinDeformationControlInterface::deformationControlPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
}	

 
void specificDeformationControlRosInit(void)
{
	//Here is where we'll declare our Ros publisher/subscribers, services, actions that doesn't fit the global architecture.
	//Beware : their scope should be this of the document. Encapsulate them in classes for example.
	return;
}

void specificDeformationControlSetSensor(std::string paramString)
{
	sensorString = paramString;//No need to mutex. Regarding how ROS work : this will be called in callback of node so after the specific funtions are called
}
