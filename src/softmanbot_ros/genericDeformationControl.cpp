#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include "std_msgs/String.h"

#include <time.h>// For sleep
#include <vector>
#include <boost/serialization/vector.hpp>

#include "genericLogic.hpp"
#include "specificDeformationControl.hpp"
#include "genericDeformationControl.hpp"


//deformationControl : participe a reconstruire l'objet avec les senseurs (noeuds intimes)

//logique : on a avec les senseurs la forme actuelle. On a la forme qu'on veut atteindre, il suffit de calculer le chemin de deformation

ros::Publisher pub;

static void supervisoryCallback(const std_msgs::Int8::ConstPtr& msg);
static void perceptionCallback(const std_msgs::String::ConstPtr& msg);

static std::stringstream sensorStringStream;
static softmanbotState currentOrder = IDLE;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "softmanbot_genericDeformationControl_ros");

	ros::NodeHandle nh;

	ROS_INFO_STREAM("Starting generic Deformation Control");

	specificDeformationControlRosInit();
	
	deformationControlInterface &defInt = getDeformationControlInterface();

	//create a publisher object
	pub = nh.advertise<std_msgs::String>("deformationControl_targetPose", 500);
	sensorStringStream.str("");
	
	//create a subscriber object
	ros::Subscriber sub = nh.subscribe("supervisor_task", 50, supervisoryCallback);
	ros::Subscriber sensorSub = nh.subscribe("specific_perception", 1000, perceptionCallback);

	//Loop at 50Hz until node shut down
	ros::Rate rate(50);	
	while(ros::ok())
	{
		try
		{
			switch(currentOrder)
			{
				case IDLE:
					defInt.deformationControlIdle();
					break;
				case INIT:
					defInt.deformationControlInit();
					break;
				case GRASPINGT1S1:
					defInt.deformationControlGraspingT1S1();					
					break;
				case PERFORMT1S1:
					defInt.deformationControlPerformT1S1();
					break;
				case GRASPINGT1S2:
					defInt.deformationControlGraspingT1S2();
					break;
				case PERFORMT1S2:
					defInt.deformationControlPerformT1S2();
					break;
				case GRASPINGT1S3:
					defInt.deformationControlGraspingT1S3();
					break;
				case PERFORMT1S3:
					defInt.deformationControlPerformT1S3();
					break;
				case GRASPINGT2S1:
					defInt.deformationControlGraspingT2S1();
					break;
				case PERFORMT2S1:
					defInt.deformationControlPerformT2S1();
					break;
				case GRASPINGT2S2:
					defInt.deformationControlGraspingT2S2();
					break;
				case PERFORMT2S2:
					defInt.deformationControlPerformT2S2();
					break;
				case GRASPINGT2S3:
					defInt.deformationControlGraspingT2S3();
					break;
				case PERFORMT2S3:
					defInt.deformationControlPerformT2S3();
					break;
				case GRASPINGT2S4:
					defInt.deformationControlGraspingT2S4();
					break;
				case PERFORMT2S4:
					defInt.deformationControlPerformT2S4();
					break;
				case GRASPINGT3S1:
					defInt.deformationControlGraspingT3S1();
					break;
				case PERFORMT3S1:
					defInt.deformationControlPerformT3S1();
					break;
				case GRASPINGT3S2:
					defInt.deformationControlGraspingT3S2();
					break;
				case PERFORMT3S2:
					defInt.deformationControlPerformT3S2();
					break;
				case GRASPINGT3S3:
					defInt.deformationControlGraspingT3S3();
					break;
				case PERFORMT3S3:
					defInt.deformationControlPerformT3S3();
					break;

				case STOP:
					defInt.deformationControlStop();
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

static void perceptionCallback(const std_msgs::String::ConstPtr& msg)
{
	//TODO : le faire passer au specific. C'est lui qui se debrouille avec.
	sensorStringStream.str(msg->data); 

	/*std::vector<pose> targetPoses;
	std::stringstream ss;
	boost::archive::text_oarchive oa(ss);
	std_msgs::String msgToSend;

	targetPoses = specificDeformationControl_multiLayerAssembly_getTargetPose(sensorStringStream.str());
	//Stream to workcell		
	oa << targetPoses;
	msgToSend.data = ss.str();
	pub.publish(msgToSend);*/	
}

/*
deformation currentShape;
ia >> currentShape; // sensor >> current shape;

//recupere l'action du superviseur
ros_get_stream(supervisor_order)
//on stop le noeud en cas d'abort, de stop?
get_current_task();
grasping :
	get_grasp_pose(currentShape);
	send_grasp(desiredGraspingPose);//genere un vecteur de pose pour le grasp
multiLayerAssembly :
	//on a deja grasp, c'est pas a nous de verifier si le grasp a laché ou pas
	std::vector<Poses> robotOrder = get_poses_from_deformation_plan(currentShape);
	stringstream ss;
	text_oarchive oa(ss);
	oa << robotOrder;
	ROS_Send(control, ss.str());
demoulding :
	//je gere ça plus tard*/
