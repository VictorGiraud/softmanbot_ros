#include "specificSupervisor.hpp"
#include <ros/ros.h>
#include <time.h>// For sleep
#include <iostream>

#include <std_msgs/Bool.h>

#include "softmanbot_ros/PLC_ReadTag.h"
#include "softmanbot_ros/PLC_WriteTag.h"

#include "PLC.hpp"

//How does it work : in Michelin's use case, we will use this to communicate with the PLC. The PLC is basically our HMI

const int k_initValue = 0X01;
const int k_initComplete = 0X01;

static ros::ServiceClient clientPlcRead;
static ros::ServiceClient clientPlcWrite;


std::string softmanbotInitNotificationTag = "INIT_TO_DO";
std::string softmanbotInitCompleteTag = "INIT_COMPLETE";

static void skilledWorkcellStepDoneCallback(const std_msgs::Bool::ConstPtr& msg);

bool isSkilledWorkcellDone = false;

static void watchdog(void);

softmanbotState state_machine(void)
{
	//Watchdog treatment
	watchdog();
	//Get State from PLC. Acknowledge to it we have the new state.
	static softmanbotState previousReadState = IDLE;

	std::cout << "call service " << std::endl;
	
	softmanbot_ros::PLC_ReadTag srv;		
	srv.request.tagName = softmanbotStatusMasterTag;
	clientPlcRead.call(srv);
	int PLCState = srv.response.valueRead;
	std::cout << "service called, got this " << int(srv.response.valueRead) << std::endl;
 
	
	if(-1 == PLCState)
	{
		//Bug de com
		PLCState = previousReadState;
	}

	softmanbot_ros::PLC_WriteTag srvWrite;		
	srvWrite.request.tagName = softmanbotStatusValidationTag;
	srvWrite.request.valueToWrite = PLCState;
	
	switch(PLCState)
	{
	case 0:
		if(IDLE != previousReadState)
		{
			clientPlcWrite.call(srvWrite);

			previousReadState = IDLE; 
		}
		return IDLE;
		break;
	case 1:
		if(INIT != previousReadState)
		{
			clientPlcWrite.call(srvWrite);
			previousReadState = INIT; 
		}
		return INIT;
		break;
	case 2:
		if(GRASPINGT1S3 != previousReadState)
		{
			clientPlcWrite.call(srvWrite);
			previousReadState = GRASPINGT1S3; 
		}
		return GRASPINGT1S3;
		break;
	case 3:
		if(PERFORMT1S3 != previousReadState)
		{
			clientPlcWrite.call(srvWrite);
			previousReadState = PERFORMT1S3; 
		}
		return PERFORMT1S3;
		break;
	case 4:
		if(STOP != previousReadState)
		{
			clientPlcWrite.call(srvWrite);
			previousReadState = STOP; 
		}
		return STOP;
		break;

	default:
		throw(std::string("exception : Invalid state required"));
		break;
	}
}

static void watchdog(void)
{

	softmanbot_ros::PLC_ReadTag srv;		
	srv.request.tagName = watchdogMasterTag;
	clientPlcRead.call(srv);
	
	int watchdogValue = srv.response.valueRead;

	softmanbot_ros::PLC_WriteTag srvWrite;		
	srvWrite.request.tagName = watchdogValidateTag;
	srvWrite.request.valueToWrite = watchdogValue;
	clientPlcWrite.call(srvWrite);
}

supervisoryInterface& getSupervisoryInterface(void)
{
	static michelinSupervisoryInterface retval;
	return retval;
}

void specificSupervisoryRosInit(void)
{
	ros::NodeHandle nh;
	ros::Subscriber subSkilledWorkcellStepDone = nh.subscribe("skilledWorkcellStepDone", 1, skilledWorkcellStepDoneCallback);
	clientPlcRead = nh.serviceClient<softmanbot_ros::PLC_ReadTag>("plc_read_tag");
	clientPlcWrite = nh.serviceClient<softmanbot_ros::PLC_WriteTag>("plc_write_tag");
}

//TODO : all this functions should check when to set step plc tag to 2
void michelinSupervisoryInterface::supervisoryInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
	if(isSkilledWorkcellDone)
	{
		isSkilledWorkcellDone = false;

		softmanbot_ros::PLC_WriteTag srvWrite;		
		srvWrite.request.tagName = softmanbotStatusStepTag;
		srvWrite.request.valueToWrite = JOBS_DONE;
		clientPlcWrite.call(srvWrite);
	}
}

void michelinSupervisoryInterface::supervisoryIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
}

void michelinSupervisoryInterface::supervisoryStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
	if(isSkilledWorkcellDone)
	{
		isSkilledWorkcellDone = false;

		softmanbot_ros::PLC_WriteTag srvWrite;		
		srvWrite.request.tagName = softmanbotStatusStepTag;
		srvWrite.request.valueToWrite = JOBS_DONE;
		clientPlcWrite.call(srvWrite);
	}
}

void michelinSupervisoryInterface::supervisoryGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
}

void michelinSupervisoryInterface::supervisoryPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
}

static void skilledWorkcellStepDoneCallback(const std_msgs::Bool::ConstPtr& msg)
{
	isSkilledWorkcellDone = true;
}

//Callback to state update from clients
	

/*void supervisory_init(void)
{

	//Wait for PLC to start	
	
	//Check at 10 Hz the PLC status	
	ros::Rate rate(1);	
	

	testCallScript();

	std::cout << "Waiting for init tag" << std::endl;


	int readValue = PLC_ReadTag(softmanbotInitNotificationTag);//TODO : je ne sais pas ce qui se passe dans le cas ou le PLC est offline/Pb de com.
		
	while(readValue != k_initValue)
	{
		rate.sleep();	
		readValue = PLC_ReadTag(softmanbotInitNotificationTag);
	}

	//call init services of all below nodes
	//TODO	
	std::cout << "Init tag Set, writing Init complete" << std::endl;

	PLC_WriteTag(softmanbotInitCompleteTag, k_initComplete);

	state_machine();//TODO : archi
	
	return;
}*/

/*
//This was actually some gripper test code
void state_machine(void) 
{
	ros::Rate rate(1);	
	while(true)	
	{
		std::cout << "Closing Gripper" << std::endl;
		PLC_WriteTag("GRIPPER_G_1", 1);
		int readValue = PLC_ReadTag("GRIPPER_G_1_STS");
		while(1 != readValue)
		{
			rate.sleep();
			readValue = PLC_ReadTag("GRIPPER_G_1_STS");
		}

		rate.sleep();
		rate.sleep();
		rate.sleep();
		rate.sleep();
		rate.sleep();		

		std::cout << "Opening Gripper" << std::endl;

		PLC_WriteTag("GRIPPER_G_1", 2);
		readValue = PLC_ReadTag("GRIPPER_G_1_STS");
		while(2 != readValue)
		{
			rate.sleep();
			readValue = PLC_ReadTag("GRIPPER_G_1_STS");
		}		

		rate.sleep();
		rate.sleep();
		rate.sleep();
		rate.sleep();
		rate.sleep();		
	}
}*/
