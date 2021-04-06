#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "std_msgs/String.h"

#include <time.h>// For sleep

#include "genericLogic.hpp"
#include "genericSkilledWorkcell.hpp"
#include "specificSkilledWorkcell.hpp"

#include <vector>
#include <boost/serialization/vector.hpp>


static std::stringstream sensorStringStream;

static void supervisoryCallback(const std_msgs::Int8::ConstPtr& msg);
static void sensorCallback(const std_msgs::String::ConstPtr& msg); 
static void deformationControlCallback(const std_msgs::String::ConstPtr& msg); 
static void perceptionControlCallback(const std_msgs::String::ConstPtr& msg); 
static void supervisoryControlCallback(const std_msgs::String::ConstPtr& msg); 

static softmanbotState currentOrder = IDLE;


int main(int argc, char **argv)
{
	ros::init(argc, argv, "softmanbot_genericSkilledWorkcell_ros");

	ros::NodeHandle nh;

	ROS_INFO_STREAM("Starting generic skilled workcell");

	
	//create a subscriber object
	ros::Subscriber subSupervisoryState		= nh.subscribe("supervisor_task", 1, supervisoryCallback);	
	ros::Subscriber subDeformationControl	= nh.subscribe("deformationControl", 1, deformationControlCallback);
	ros::Subscriber subPerceptionControl	= nh.subscribe("perceptionControl", 1, perceptionControlCallback);
	ros::Subscriber subSupervisoryControl	= nh.subscribe("supervisoryControl", 1, supervisoryControlCallback);
	
	ros::Subscriber specificSensorSub  = nh.subscribe("specific_perception", 1, sensorCallback);


	specificSkilledWorkcellRosInit();
	
	skilledWorkcellInterface &skillInt = getSkilledWorkcellInterface();

	//Loop at 50Hz until node shut down
	ros::Rate rate(50);	
	while(ros::ok())
	{
		try
		{
			switch(currentOrder)
			{
				case IDLE:
					skillInt.skilledWorkcellIdle();
					break;
				case INIT:
					skillInt.skilledWorkcellInit();
					break;
				case GRASPINGT1S1:
					skillInt.skilledWorkcellGraspingT1S1();					
					break;
				case PERFORMT1S1:
					skillInt.skilledWorkcellPerformT1S1();
					break;
				case GRASPINGT1S2:
					skillInt.skilledWorkcellGraspingT1S2();
					break;
				case PERFORMT1S2:
					skillInt.skilledWorkcellPerformT1S2();
					break;
				case GRASPINGT1S3:
					skillInt.skilledWorkcellGraspingT1S3();
					break;
				case PERFORMT1S3:
					skillInt.skilledWorkcellPerformT1S3();
					break;
				case GRASPINGT2S1:
					skillInt.skilledWorkcellGraspingT2S1();
					break;
				case PERFORMT2S1:
					skillInt.skilledWorkcellPerformT2S1();
					break;
				case GRASPINGT2S2:
					skillInt.skilledWorkcellGraspingT2S2();
					break;
				case PERFORMT2S2:
					skillInt.skilledWorkcellPerformT2S2();
					break;
				case GRASPINGT2S3:
					skillInt.skilledWorkcellGraspingT2S3();
					break;
				case PERFORMT2S3:
					skillInt.skilledWorkcellPerformT2S3();
					break;
				case GRASPINGT2S4:
					skillInt.skilledWorkcellGraspingT2S4();
					break;
				case PERFORMT2S4:
					skillInt.skilledWorkcellPerformT2S4();
					break;
				case GRASPINGT3S1:
					skillInt.skilledWorkcellGraspingT3S1();
					break;
				case PERFORMT3S1:
					skillInt.skilledWorkcellPerformT3S1();
					break;
				case GRASPINGT3S2:
					skillInt.skilledWorkcellGraspingT3S2();
					break;
				case PERFORMT3S2:
					skillInt.skilledWorkcellPerformT3S2();
					break;
				case GRASPINGT3S3:
					skillInt.skilledWorkcellGraspingT3S3();
					break;
				case PERFORMT3S3:
					skillInt.skilledWorkcellPerformT3S3();
					break;

				case STOP:
					skillInt.skilledWorkcellStop();
					break;

				default:
				throw(std::string("Unexpected state in supervisor state machine"));
				
			}
		}
		catch(std::string exc)
		{
			std::cout << "caught this : " << exc << std::endl;
			exit(EXIT_FAILURE);
		}
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		ros::spinOnce();
		rate.sleep();
	}
}

static void supervisoryCallback(const std_msgs::Int8::ConstPtr& msg)
{
	currentOrder = softmanbotState(msg->data);
}

static void deformationControlCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("DeformationControl CB");

	if(DEFORMATION_CONTROL == specificSkilledWorkcellGetMaster())
	{
		/*std::stringstream ss(msg->data);
		boost::archive::text_iarchive ia(ss);
	
		std::vector<pose> targetPoses;
		std::vector<pose> currentStepPoses;
	
		ia >> targetPoses;*/
		std::string buffer = msg->data;//TODO clean
		specificSkilledWorkcellControl(buffer, sensorStringStream.str());
	
		/*std::stringstream ssSend;
		boost::archive::text_oarchive oa(ssSend);
		std_msgs::String msgToSend;
		oa << currentStepPoses;
		msgToSend.data = ssSend.str();
		currentRobotPosPub.publish(msgToSend);*/
		//Stream de la targetPose recuperee par le genericSKilledWorkcell
	}
}

static void perceptionControlCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("PerceptionControl CB");
	if(PERCEPTION == specificSkilledWorkcellGetMaster())
	{	
		std::string buffer = msg->data;//TODO clean
		specificSkilledWorkcellControl(buffer, sensorStringStream.str());
	}
}

static void supervisoryControlCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("SupervisorControl CB");
	if(SUPERVISOR == specificSkilledWorkcellGetMaster())
	{
		std::string buffer = msg->data;//TODO clean
		specificSkilledWorkcellControl(buffer, sensorStringStream.str());
	}
}


static void sensorCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("Sensor CB");

	sensorStringStream.str(msg->data);
	specificSkilledWorkcell_setSensor(msg->data);
}
