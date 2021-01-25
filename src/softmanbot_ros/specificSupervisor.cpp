#include "specificSupervisor.hpp"
#include <ros/ros.h>
#include <time.h>// For sleep
#include <iostream>

#include <std_msgs/Bool.h>

int get_state(void);

softmanbotState previousReadState = IDLE; 

softmanbotState state_machine(void)
{
	
	//Get State from External source (IHM Or PLC)
 	
	int PLCState = get_state();
 	
	switch(PLCState)
	{
	case 0:
		if(IDLE != previousReadState)
		{
			previousReadState = IDLE; 
		}
		return IDLE;
		break;
	case 1:
		if(INIT != previousReadState)
		{
			previousReadState = INIT; 
		}
		return INIT;
		break;
	case 2:
		if(GRASPINGT1S3 != previousReadState)
		{
			previousReadState = GRASPINGT1S3; 
		}
		return GRASPINGT1S3;
		break;
	case 3:
		if(PERFORMT1S3 != previousReadState)
		{
			previousReadState = PERFORMT1S3; 
		}
		return PERFORMT1S3;
		break;
	case 4:
		if(STOP != previousReadState)
		{
			previousReadState = STOP; 
		}
		return STOP;
		break;

	default:
		throw(std::string("exception : Invalid state required"));
		break;
	}
}


supervisoryInterface& getSupervisoryInterface(void)
{
	static michelinSupervisoryInterface retval;
	return retval;
}

void specificSupervisoryRosInit(void)
{

}

void michelinSupervisoryInterface::supervisoryInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
}

void michelinSupervisoryInterface::supervisoryIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
}

void michelinSupervisoryInterface::supervisoryStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
}

void michelinSupervisoryInterface::supervisoryGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
}

void michelinSupervisoryInterface::supervisoryPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
}

int get_state(void)
{
	return 0;
}

