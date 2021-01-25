#include <ros/ros.h>
#include <std_msgs/Int8.h>

#include <time.h>// For sleep

#include "genericLogic.hpp"
#include "genericSupervisor.hpp"
#include "specificSupervisor.hpp"


softmanbotState	TaskPlanner_update			(void);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "softmanbot_supervisor_ros");

	ros::NodeHandle nh;

	ROS_INFO_STREAM("Starting SoftManBot Supervisor");
	specificSupervisoryRosInit();
	
	supervisoryInterface& supInt = getSupervisoryInterface();
	
	//create a publisher object
	ros::Publisher pub = nh.advertise<std_msgs::Int8>("supervisor_task", 50);
	softmanbotState currentTask;
	
	//Loop at 50Hz until node shut down
	ros::Rate rate(50);	
	while(ros::ok())
	{
		//Get current task, advertise it to other nodes
		currentTask = TaskPlanner_update();
		std::stringstream ss;
    	std_msgs::Int8 msg;
		msg.data = currentTask;
		pub.publish(msg);		

		try
		{
			switch(currentTask)
			{
				case IDLE:
					supInt.supervisoryIdle();
					break;
				case INIT:
					supInt.supervisoryInit();
					break;
				case GRASPINGT1S1:
					supInt.supervisoryGraspingT1S1();					
					break;
				case PERFORMT1S1:
					supInt.supervisoryPerformT1S1();
					break;
				case GRASPINGT1S2:
					supInt.supervisoryGraspingT1S2();
					break;
				case PERFORMT1S2:
					supInt.supervisoryPerformT1S2();
					break;
				case GRASPINGT1S3:
					supInt.supervisoryGraspingT1S3();
					break;
				case PERFORMT1S3:
					supInt.supervisoryPerformT1S3();
					break;
				case GRASPINGT2S1:
					supInt.supervisoryGraspingT2S1();
					break;
				case PERFORMT2S1:
					supInt.supervisoryPerformT2S1();
					break;
				case GRASPINGT2S2:
					supInt.supervisoryGraspingT2S2();
					break;
				case PERFORMT2S2:
					supInt.supervisoryPerformT2S2();
					break;
				case GRASPINGT2S3:
					supInt.supervisoryGraspingT2S3();
					break;
				case PERFORMT2S3:
					supInt.supervisoryPerformT2S3();
					break;
				case GRASPINGT2S4:
					supInt.supervisoryGraspingT2S4();
					break;
				case PERFORMT2S4:
					supInt.supervisoryPerformT2S4();
					break;
				case GRASPINGT3S1:
					supInt.supervisoryGraspingT3S1();
					break;
				case PERFORMT3S1:
					supInt.supervisoryPerformT3S1();
					break;
				case GRASPINGT3S2:
					supInt.supervisoryGraspingT3S2();
					break;
				case PERFORMT3S2:
					supInt.supervisoryPerformT3S2();
					break;
				case GRASPINGT3S3:
					supInt.supervisoryGraspingT3S3();
					break;
				case PERFORMT3S3:
					supInt.supervisoryPerformT3S3();
					break;

				case STOP:
					supInt.supervisoryStop();
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
		rate.sleep();
	}
}

softmanbotState TaskPlanner_update(void)
{
	softmanbotState currentTask;
	try 	
	{	
		currentTask = state_machine();
	}
	catch(std::string exc)
	{
		std::cout << "caught this : " << exc << std::endl;
		exit(EXIT_FAILURE);		
	}
	return currentTask;	
}
