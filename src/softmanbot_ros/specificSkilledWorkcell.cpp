#include "specificSkilledWorkcell.hpp"
#include "genericLogic.hpp"
#include <ros/ros.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <sstream>
#include <assert.h>
#include <stdint.h>

#include <cmath>
#include <SDL2/SDL.h>

#include "zhelpers.hpp"

#include "PLC.hpp"
#include <string>

#include <thread>
#include <chrono>

#include <std_msgs/Bool.h>

#include "softmanbot_ros/PLC_ReadTag.h"
#include "softmanbot_ros/PLC_WriteTag.h"

#include "PoseRPY.h"


void testZmqPub(void);
static void testDualArm(void);
//static SDL_Rect pointsToRect(Point pointToTransform);


static void gripperOpen(int gripperNumber);
static void gripperClose(int gripperNumber);

campero_ur_ip_controllers::PoseRPY armGetOrder(pose targetPose);

static ros::Publisher pubUR10Arm1;
static ros::Publisher pubUR10Arm2;

static softmanbotState lastOrder = IDLE;//I want to do interfaces actions once, rest is on callbacks. Thus, I have to keep memory of last action treated 

static std::string arm1RestJointOrder("rostopic pub -1 Arm1/joint_array_control/command std_msgs/Float64MultiArray 'data: [0.05, -0.27, -2.81, -1.58, 0.02, 1.40]'");
static std::string arm1IntermediaryJointOrder("rostopic pub -1 Arm1/joint_array_control/command std_msgs/Float64MultiArray 'data: [-0.32, -1.10, -1.67, 0.21, 0.3, 0.9]'");
static std::string arm1WorkJointOrder("rostopic pub -1 Arm1/joint_array_control/command std_msgs/Float64MultiArray 'data: [-0.65, -1.23, -1.62, 0.0, 0.6, 1.3]'");

static std::string arm2RestJointOrder("rostopic pub -1 Arm/joint_array_control/command std_msgs/Float64MultiArray 'data: [0.05, -2.78, 2.71, -1.18, 0.03, 1.24]'");
static std::string arm2IntermediaryJointOrder("rostopic pub -1 /joint_array_control/command std_msgs/Float64MultiArray 'data: [0.048, -2.6, 2.2, -2.2, 0.06, 1.93]'");
static std::string arm2WorkJointOrder("rostopic pub -1 /joint_array_control/command std_msgs/Float64MultiArray 'data: [0.26, -2.05, 1.51, -2.6, -0.3, 1.74]'");

static ros::Publisher pubSkilledWorkcellStatus;

static void notifySupervisorTaskDone(void);

bool plc_read_tag_callback(softmanbot_ros::PLC_ReadTag::Request &req, softmanbot_ros::PLC_ReadTag::Response &res);
bool plc_write_tag_callback(softmanbot_ros::PLC_WriteTag::Request &req, softmanbot_ros::PLC_WriteTag::Response &res);


class serviceHandler
{
public:
	ros::ServiceServer servicePlcRead;
	ros::ServiceServer servicePlcWrite;
};

serviceHandler serviceScoped;



typedef enum
{
	ARM1,
	ARM2
}currentRobotArm;

typedef enum
{
	NO_CONTROLLER,
	SPEED_CONTROLLER,
	POSITION_CONTROLLER,
	JOINT_CONTROLLER	
}currentRobotController; //Il en faudra un pour le bras droit et un pour le gauche


typedef enum
{
	GRIPPER_IDLE,//Necessaire pour Nicolas Bard afin de n'etre pas obige d'effectuer une action de pince
	GRIPPER_CLOSE,
	GRIPPER_OPEN,
}gripper_status;

static currentRobotController arm1CurrentController = NO_CONTROLLER;
static currentRobotController arm2CurrentController = NO_CONTROLLER;

void stopCurrentController(currentRobotArm armToStop);
void startCurrentController(currentRobotArm armToStart, currentRobotController controllerToStart);

static nodeID currentMaster = SKILLED_WORKCELL;


const std::string gripperTagNames[] = {"GRIPPER_G_1", "GRIPPER_G_2", "GRIPPER_G_3", "GRIPPER_D_1", "GRIPPER_D_2", "GRIPPER_D_3"};
const int k_maxTagNumber = 6;

static void notifySupervisorTaskDone(void)
{
	ros::NodeHandle nh;
	pubSkilledWorkcellStatus = nh.advertise<std_msgs::Bool>("skilledWorkcellStepDone", 1);
	std_msgs::Bool msgToSend;
	msgToSend.data = true;
	pubSkilledWorkcellStatus.publish(msgToSend);
}


void stopCurrentController(currentRobotArm armToStop)
{
	std::string pathToCall = "./src/softmanbot_ros/specificMichelin/scripts/";
	currentRobotController currentArmController;	
	if(ARM1 == armToStop)
	{
		pathToCall.append("arm1_");
		currentArmController = arm1CurrentController;
		arm1CurrentController = NO_CONTROLLER;
	}
	else
	{
		assert(ARM2 == armToStop);
		pathToCall.append("arm2_");
		currentArmController = arm2CurrentController;
		arm2CurrentController = NO_CONTROLLER;
	}

	switch(currentArmController)
	{
		case NO_CONTROLLER:
			return;
			break;
		case SPEED_CONTROLLER:			
			pathToCall.append("stop_vel_based_cartesian_velocity_controller.sh");
			break;
		case POSITION_CONTROLLER:
			pathToCall.append("stop_cartesian_velocity_controller.sh");			
			break;
		case JOINT_CONTROLLER:
			pathToCall.append("stop_joint_controller.sh");						
			break;
	}
	std::cout << "calling controller stop" << pathToCall.c_str() << std::endl;
	system(pathToCall.c_str());
	return;
}

void startCurrentController(currentRobotArm armToStart, currentRobotController controllerToStart)
{
	std::string pathToCall = "./src/softmanbot_ros/specificMichelin/scripts/";
	currentRobotController currentArmController;	
	if(ARM1 == armToStart)
	{
		pathToCall.append("arm1_");
		if(controllerToStart != arm1CurrentController)
		{
			arm1CurrentController = controllerToStart;
		}
		else
		{
			return;
		}
	}
	else
	{
		assert(ARM2 == armToStart);
		pathToCall.append("arm2_");
		if(controllerToStart != arm1CurrentController)
		{
			arm2CurrentController = controllerToStart;
		}
		else
		{
			return;
		}
	}

	switch(controllerToStart)
	{
		case NO_CONTROLLER:
			return;
			break;
		case SPEED_CONTROLLER:			
			pathToCall.append("start_vel_based_cartesian_velocity_controller.sh");
			break;
		case POSITION_CONTROLLER:
			pathToCall.append("start_cartesian_velocity_controller.sh");			
			break;
		case JOINT_CONTROLLER:
			pathToCall.append("start_joint_controller.sh");						
			break;
	}
	std::cout << "calling controller start" << pathToCall.c_str() << std::endl;

	system(pathToCall.c_str());
	return;
}


skilledWorkcellInterface& getSkilledWorkcellInterface(void)
{
	static michelinSkilledWorkcellInterface retval;
	return retval;
}

void michelinSkilledWorkcellInterface::skilledWorkcellInit(void)
{
	/*static bool isArm1IntermediaryPositionReached = false;
	static bool isArm2IntermediaryPositionReached = false;

	static bool isArm1WorkPositionReached = false;
	static bool isArm2WorkPositionReached = false;*/

	if(lastOrder != INIT)
	{
		std::cout << "Call polymorphed version of the interface : Init" << std::endl;	
		lastOrder = INIT;
		
		currentMaster = SKILLED_WORKCELL;
		
		/*isArm1IntermediaryPositionReached = false;
		isArm2IntermediaryPositionReached = false;
		isArm1WorkPositionReached = false;
		isArm2WorkPositionReached = false;*/

		std::cout << "test stop controlleur" << std::endl;	
		stopCurrentController(ARM1);
		stopCurrentController(ARM2);
		std::cout << "reussite stop controlleur" << std::endl;	

		std::cout << "test gripper open" << std::endl;					
		//gripperOpen(0);
		//gripperOpen(3);		
		std::cout << "reussite gripper open" << std::endl;	
		

		std::cout << "test start controlleur" << std::endl;	
		startCurrentController(ARM1, JOINT_CONTROLLER);
		startCurrentController(ARM2, JOINT_CONTROLLER);
		//Wait a bit so the scripts stop and starts before proceeding
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		//Send the order to move to position
		system(arm1IntermediaryJointOrder.c_str());
		system(arm2IntermediaryJointOrder.c_str());
		//Wait a bit so the scripts stop and starts before proceeding
 		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
		//Send the order to move to position
		system(arm1IntermediaryJointOrder.c_str());
		system(arm2IntermediaryJointOrder.c_str());
		//Wait a bit
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  	
		//Send the order to move to work position
		system(arm1WorkJointOrder.c_str());
		system(arm2WorkJointOrder.c_str());

		//Wait a bit
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//Notify supervisor we're ok			
		notifySupervisorTaskDone();
	}
	
	/*if(!isArm1IntermediaryPositionReached)
	{
		float delta;
		//Calculate Delta
		//if(delta < thresh) { isArm1IntermediaryPositionReached = true;}
		//Send the order to move to work position

	}
	else
	{
		if(!isArm1WorkPositionReached)
		{
			float delta;
			//Calculate Delta
			//if(delta < thresh) { isArm1WorkPositionReached = true;}
		}
	}
	if(!isArm2IntermediaryPositionReached)
	{
		float delta;
		//Calculate Delta
		//if(delta < thresh) { isArm2IntermediaryPositionReached = true;}
		//Send the order to move to work position
	}
	else
	{
		if(!isArm2WorkPositionReached)
		{
			float delta;
			//Calculate Delta
			//if(delta < thresh) { isArm2WorkPositionReached = true;}
		}

	}
	if(isArm1WorkPositionReached && isArm2WorkPositionReached)
	{
		//Notify supervisory skilledWorkcell Init is done
	}*/
}

void michelinSkilledWorkcellInterface::skilledWorkcellIdle(void)
{
	if(lastOrder != IDLE)
	{
		std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
		lastOrder = IDLE;
	}
}

void michelinSkilledWorkcellInterface::skilledWorkcellStop(void)
{
	/*static bool isArm1IntermediaryPositionReached = false;
	static bool isArm2IntermediaryPositionReached = false;

	static bool isArm1RestPositionReached = false;
	static bool isArm2RestPositionReached = false;*/

	if(lastOrder != STOP)
	{
		std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
		currentMaster = SKILLED_WORKCELL;
		
		/*isArm1IntermediaryPositionReached = false;
		isArm2IntermediaryPositionReached = false;
		isArm1RestPositionReached = false;
		isArm2RestPositionReached = false;*/

		stopCurrentController(ARM1);
		stopCurrentController(ARM2);
		
		gripperOpen(0);
		gripperOpen(3);		

		startCurrentController(ARM1, JOINT_CONTROLLER);
		startCurrentController(ARM2, JOINT_CONTROLLER);
		//Wait a bit so the scripts stop and starts before proceeding
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		//Send the order to move to position
		system(arm1IntermediaryJointOrder.c_str());
		system(arm2IntermediaryJointOrder.c_str());
		//Wait a bit
 		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  		//Send the order to move to rest position
		system(arm1RestJointOrder.c_str());
		system(arm2RestJointOrder.c_str());

		//Wait a bit
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		//Notify supervisor we're ok
		notifySupervisorTaskDone();			
	}
	
	/*if(!isArm1IntermediaryPositionReached)
	{
		float delta;
		//Calculate Delta
		//if(delta < thresh) { isArm1IntermediaryPositionReached = true;}
		//Send the order to move to work position

	}
	else
	{
		if(!isArm1RestPositionReached)
		{
			float delta;
			//Calculate Delta
			//if(delta < thresh) { isArm1WorkPositionReached = true;}
		}
	}
	if(!isArm2IntermediaryPositionReached)
	{
		float delta;
		//Calculate Delta
		//if(delta < thresh) { isArm2IntermediaryPositionReached = true;}
		//Send the order to move to work position
	}
	else
	{
		if(!isArm2RestPositionReached)
		{
			float delta;
			//Calculate Delta
			//if(delta < thresh) { isArm2WorkPositionReached = true;}
		}

	}
	if(isArm1RestPositionReached && isArm2RestPositionReached)
	{
		gripperClose(0);
		gripperClose(3);		

		//Notify supervisory skilledWorkcell Init is done
	}*/
}

void michelinSkilledWorkcellInterface::skilledWorkcellGraspingT1S3(void)
{
	if(lastOrder != GRASPINGT1S3)
	{
		std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
		lastOrder = GRASPINGT1S3;
	}
}

void michelinSkilledWorkcellInterface::skilledWorkcellPerformT1S3(void)
{
	if(lastOrder != PERFORMT1S3)
	{
		std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
		lastOrder = PERFORMT1S3;
		//Stop current controllers
		//Start vec_cartesian controllers
		//Allows orders from Shape Control
	}	
}

nodeID specificSkilledWorkcellGetMaster(void)
{
	return currentMaster;
}
/*void specificSkilledWorkcell_init(void)
{
	std::cout << "specificSkilledWorkcell init" << std::endl;


	//pubUR10Arm1 = nh.advertise<campero_ur_ip_controllers::PoseRPY>("/test1", 500);	
	//pubUR10Arm2 = nh.advertise<campero_ur_ip_controllers::PoseRPY>("/test2", 500);	


	//TODO : 
	//		 creer un objet qui gere chacun des bras.
	//		 l'objet va prendre le nom du namespace du bras, le customPort, le reversePort. Pas besoin de l'IP, le controlleur UR sera lance par le roslaunch a l'init du systeme.
	//			Indiquer aussi la position relative du bras % le referentiel 0 du projet. Tout ça peut probablement rentrer dans un fichier de conf, 
	//			ainsi que les "points articulaires" de passage du robot au démarage 
	
	//		 Parametre prive de l'objet : 	- le controlleur couramment lancé, pour le switcher
	//										- Si il y a un ordre en cours
	// 		 methodes publiques : cartesian_position_control(cartesianPosition);
	//							  cartesian_velocity_control(cartesianVelocity)
	//		 methodes privees : joint_control(jointPosition). Privé parce que je vais m'en servir que à l'init pour l'instant.


	//		 1 - Advertise pour chacun des deux bras
	//		 2 - creer une fonction qui permet d'actualiser la position courante des bras robots
	
	//Switch to joint controller
	//Check which of the "path point" is the closer to current configuration
	//Go to this path point if epsilon
	//launch	

	//TODO : pour l'instant je suis juste en position. Je vais vouloir switch entre les controlleurs position/vitesse et joint. Je peux utiliser system() pour cela.

	testDualArm();

	return;
}*/

std::vector<pose> specificSkilledWorkcellControl(std::vector<pose> targetPoses, std::string sensorString)
{
	std::vector<pose> retval;	
	std::stringstream sensorStringStream(sensorString);
	
	pose firstArmTargetPose = targetPoses[0];
	
	/*campero_ur_ip_controllers::PoseRPY msgToSend;
	msgToSend.id = 0;
	msgToSend.position.x = firstArmTargetPose.x;
	msgToSend.position.y = firstArmTargetPose.y;
	msgToSend.position.z = firstArmTargetPose.z;
	msgToSend.orientation.roll = firstArmTargetPose.roll;
	msgToSend.orientation.pitch = firstArmTargetPose.pitch;
	msgToSend.orientation.yaw = firstArmTargetPose.yaw;		

	pubUR10.publish(msgToSend);*/


	retval.push_back(firstArmTargetPose);//What am I even supposed to return?

	std::cout << "controlling robot" << std::endl;

	return retval;
}


void gripperOpen(int gripperNumber)
{
	if((gripperNumber >= 0) && (gripperNumber < k_maxTagNumber))
	{
		PLC_WriteTag(gripperTagNames[gripperNumber], GRIPPER_OPEN);
	}	
}

void gripperClose(int gripperNumber)
{
	if((gripperNumber >= 0) && (gripperNumber < k_maxTagNumber))
	{
		PLC_WriteTag(gripperTagNames[gripperNumber], GRIPPER_CLOSE);
	}	
}


//TODO : fonction qui check le statut du gripper

/*static SDL_Rect pointsToRect(Point pointToTransform)
{
	SDL_Rect retval;

	int originX = SCREEN_WIDTH/2;
	int originY = SCREEN_HEIGHT/2;

	retval.x = 10 * pointToTransform.getCoordinateX() + originX - 3;
	retval.y = - 10 * pointToTransform.getCoordinateY() + originY - 3;
	retval.w = 5;
	retval.h = 5;

	return retval;
}*/

void testZmqPub(void)
{
	zmq::context_t context(1);
    zmq::socket_t publisher(context, ZMQ_PUB);
    publisher.bind("tcp://*:5563");

    while (1) {
        //  Write two messages, each with an envelope and content
        s_sendmore (publisher, "A");
        s_send (publisher, "We don't want to see this");
        s_sendmore (publisher, "B");
        s_send (publisher, "We would like to see this");
        sleep (1);
    }
    return;
}

static void testDualArm(void)
{
	pose arm1BasePose;
	pose arm2BasePose;

	pose arm1Target1Pose;
	pose arm2Target1Pose;

	pose arm1Target2Pose;
	pose arm2Target2Pose;

	pose arm1Target3Pose;
	pose arm2Target3Pose;

	//Init
	ros::NodeHandle nh;
	pubUR10Arm1 = nh.advertise<campero_ur_ip_controllers::PoseRPY>("/Arm1/cartesian_velocity_control/command", 500);	
	pubUR10Arm2 = nh.advertise<campero_ur_ip_controllers::PoseRPY>("/Arm2/cartesian_velocity_control/command", 500);	

	arm1BasePose.x = -0.35;
	arm1BasePose.y = 0;
	arm1BasePose.z = 0.7;
	arm1BasePose.roll = -2.0;
	arm1BasePose.pitch = 1.6;
	arm1BasePose.yaw = -0.45;

	arm1Target1Pose.x = -0.35;
	arm1Target1Pose.y = 0;
	arm1Target1Pose.z = 0.95;
	arm1Target1Pose.roll = -2.0;
	arm1Target1Pose.pitch = 1.6;
	arm1Target1Pose.yaw = -0.45;

	arm1Target2Pose.x = -0.35;
	arm1Target2Pose.y = 0.22;
	arm1Target2Pose.z = 0.95;
	arm1Target2Pose.roll = -2.0;
	arm1Target2Pose.pitch = 1.6;
	arm1Target2Pose.yaw = -0.45;

	arm1Target3Pose.x = -0.35;
	arm1Target3Pose.y = 0.22;
	arm1Target3Pose.z = 0.7;
	arm1Target3Pose.roll = -2.0;
	arm1Target3Pose.pitch = 1.6;
	arm1Target3Pose.yaw = -0.45;


	arm2BasePose.x = 0.35;
	arm2BasePose.y = 0;
	arm2BasePose.z = 0.7;
	arm2BasePose.roll = -1.28;
	arm2BasePose.pitch = 1.55;
	arm2BasePose.yaw = 0.25;

	arm2Target1Pose.x = 0.35;
	arm2Target1Pose.y = 0;
	arm2Target1Pose.z = 0.95;
	arm2Target1Pose.roll = -1.28;
	arm2Target1Pose.pitch = 1.55;
	arm2Target1Pose.yaw = 0.25;

	arm2Target2Pose.x = 0.35;
	arm2Target2Pose.y = 0.22;
	arm2Target2Pose.z = 0.95;
	arm2Target2Pose.roll = -1.28;
	arm2Target2Pose.pitch = 1.55;
	arm2Target2Pose.yaw = 0.25;

	arm2Target3Pose.x = 0.35;
	arm2Target3Pose.y = 0.22;
	arm2Target3Pose.z = 0.7;
	arm2Target3Pose.roll = -1.28;
	arm2Target3Pose.pitch = 1.55;
	arm2Target3Pose.yaw = 0.25;

	gripperOpen(0);
	gripperOpen(3);


	int i = 0;	
	
	//Loop at 50Hz until node shut down
	ros::Rate rate(1);	
	while(ros::ok())
	{

		rate.sleep();
		i++;
		if(i >= 22)
		{
			i = 0;
		}

		
		if(i >= 17)
		{
			pubUR10Arm1.publish(armGetOrder(arm1Target3Pose));
			pubUR10Arm2.publish(armGetOrder(arm2Target3Pose));			
		}
		else
		{
			if(i >= 10)
			{
				if(i == 16)
				{
					gripperClose(0);
					gripperClose(3);
				}				
	
				pubUR10Arm1.publish(armGetOrder(arm1Target2Pose));
				pubUR10Arm2.publish(armGetOrder(arm2Target2Pose));			
			}
			else
			{

				if(i >= 5)
				{
					pubUR10Arm1.publish(armGetOrder(arm1Target1Pose));
					pubUR10Arm2.publish(armGetOrder(arm2Target1Pose));			
				}
				else
				{
					if(i == 0)
					{
						gripperOpen(0);
						gripperOpen(3);
					}
					pubUR10Arm1.publish(armGetOrder(arm1BasePose));
					pubUR10Arm2.publish(armGetOrder(arm2BasePose));			
				}
			}
		}
		ros::spinOnce();
	}

	return;
}



campero_ur_ip_controllers::PoseRPY armGetOrder(pose targetPose)
{	
	campero_ur_ip_controllers::PoseRPY msgToSend;
	msgToSend.id = 0;
	msgToSend.position.x = targetPose.x;
	msgToSend.position.y = targetPose.y;
	msgToSend.position.z = targetPose.z;
	msgToSend.orientation.roll = targetPose.roll;
	msgToSend.orientation.pitch = targetPose.pitch;
	msgToSend.orientation.yaw = targetPose.yaw;

	return msgToSend;
}

bool plc_read_tag_callback(softmanbot_ros::PLC_ReadTag::Request &req, softmanbot_ros::PLC_ReadTag::Response &res)
{
	std::cout << "called read service" << std::endl;
	//res.valueRead = PLC_ReadTag(req.tagName);
	res.valueRead = 0;	
	return true;
}

bool plc_write_tag_callback(softmanbot_ros::PLC_WriteTag::Request &req, softmanbot_ros::PLC_WriteTag::Response &res)
{
	std::cout << "called write service" << std::endl;
	//PLC_WriteTag(req.tagName, req.valueToWrite);
	res.ret = 1;
	return true;
}

void specificSkilledWorkcellRosInit(void)
{
	PLC_Init();
	ROS_INFO("setting up services");

	ros::NodeHandle nh;

	serviceScoped.servicePlcRead = nh.advertiseService("plc_read_tag", plc_read_tag_callback);
    serviceScoped.servicePlcWrite = nh.advertiseService("plc_write_tag", plc_write_tag_callback);
	
	ros::spinOnce();
	ROS_INFO("services ready"); 
	return;
}

