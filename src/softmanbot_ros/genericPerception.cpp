#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "std_msgs/String.h"

#include <time.h>// For sleep


#include "specificPerception.hpp"
#include "genericLogic.hpp"

//Perception : Comment est l'environnement. Grasp en intimite avec l'actionneur. Reconstruction de la forme en lien avec la deformation

static void supervisoryCallback			(const std_msgs::Int8::ConstPtr& msg);

static softmanbotState currentOrder = IDLE;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "softmanbot_genericPerception_ros");

	ros::NodeHandle nh;

	ROS_INFO_STREAM("Starting generic perception");

	specificPerceptionRosInit();

	//create a publisher object
	//ros::Publisher pub = nh.advertise<std_msgs::string>("deformationControl_targetPose", 50);
	perceptionInterface &percInt = getPerceptionInterface();
	
	//create a subscriber object : supervisor order
	ros::Subscriber subSupervisoryState = nh.subscribe("supervisor_task", 50, supervisoryCallback);


	//Loop at 2Hz until node shut down
	ros::Rate rate(2);	
	while(ros::ok())
	{		
		try
		{
			switch(currentOrder)
			{
				case IDLE:
					percInt.perceptionIdle();
					break;
				case INIT:
					percInt.perceptionInit();
					break;
				case GRASPINGT1S1:
					percInt.perceptionGraspingT1S1();					
					break;
				case PERFORMT1S1:
					percInt.perceptionPerformT1S1();
					break;
				case GRASPINGT1S2:
					percInt.perceptionGraspingT1S2();
					break;
				case PERFORMT1S2:
					percInt.perceptionPerformT1S2();
					break;
				case GRASPINGT1S3:
					percInt.perceptionGraspingT1S3();
					break;
				case PERFORMT1S3:
					percInt.perceptionPerformT1S3();
					break;
				case GRASPINGT2S1:
					percInt.perceptionGraspingT2S1();
					break;
				case PERFORMT2S1:
					percInt.perceptionPerformT2S1();
					break;
				case GRASPINGT2S2:
					percInt.perceptionGraspingT2S2();
					break;
				case PERFORMT2S2:
					percInt.perceptionPerformT2S2();
					break;
				case GRASPINGT2S3:
					percInt.perceptionGraspingT2S3();
					break;
				case PERFORMT2S3:
					percInt.perceptionPerformT2S3();
					break;
				case GRASPINGT2S4:
					percInt.perceptionGraspingT2S4();
					break;
				case PERFORMT2S4:
					percInt.perceptionPerformT2S4();
					break;
				case GRASPINGT3S1:
					percInt.perceptionGraspingT3S1();
					break;
				case PERFORMT3S1:
					percInt.perceptionPerformT3S1();
					break;
				case GRASPINGT3S2:
					percInt.perceptionGraspingT3S2();
					break;
				case PERFORMT3S2:
					percInt.perceptionPerformT3S2();
					break;
				case GRASPINGT3S3:
					percInt.perceptionGraspingT3S3();
					break;
				case PERFORMT3S3:
					percInt.perceptionPerformT3S3();
					break;

				case STOP:
					percInt.perceptionStop();
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
		rate.sleep();
	}
}

static void supervisoryCallback(const std_msgs::Int8::ConstPtr& msg)
{
	currentOrder = softmanbotState(msg->data);
}
