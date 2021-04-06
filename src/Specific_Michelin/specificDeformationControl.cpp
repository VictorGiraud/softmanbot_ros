#include "specificDeformationControl.hpp"
#include <boost/archive/text_iarchive.hpp>	

#include <sstream>
#include <fstream>

#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


#include "genericLogic.hpp"
#include <ros/ros.h>

#include "zhelpers.hpp"

#include "Eigen/Dense"
#include "positionConverter.hpp"

#include <chrono>

static std::string sensorString;

static softmanbotState lastOrder = IDLE; 

static void notifySupervisorTaskDone(void);

const int NopainNogain = 10;


class pubHandler
{
public:
	ros::Publisher pubExternalControl;
	ros::Publisher pubDeformationControlStatus;

	ros::Publisher gripperOpen;
	ros::Publisher gripperClose;
};

pubHandler pubScoped;


constexpr uint8_t PID_HORIZON = 10;

class PID
{
public:
	PID(float kP, float kI, float kD):P(kP), I(kI), D(kD), lastTimeError(0), currentError(0), currentErrorIndex(0), lastError{0} {};
	~PID(){};
	void setCurrentError(float error)
	{
		lastTimeError = currentError;
		lastError[currentErrorIndex] = lastTimeError;
		currentErrorIndex++;
		currentErrorIndex %= PID_HORIZON;
	};
	float getCorrection(void)
	{
		float cumulativeError;
		for(int i = 0; i < PID_HORIZON; i++)
		{
			cumulativeError += lastError[i];
		}
		cumulativeError = cumulativeError/ PID_HORIZON;
		float correction = 	-(P * currentError + I * cumulativeError + D*(currentError - lastTimeError));		
		return correction;
	}
private:
	float lastError[PID_HORIZON];
	float lastTimeError;
	float currentError;	
	float P;
	float I;
	float D;
	int currentErrorIndex;
};



deformationControlInterface& getDeformationControlInterface(void)
{
	static michelinDeformationControlInterface retval;
	return retval;
}

void michelinDeformationControlInterface::deformationControlInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
	lastOrder = INIT;
}

void michelinDeformationControlInterface::deformationControlIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
	lastOrder = IDLE;
}

void michelinDeformationControlInterface::deformationControlStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
	lastOrder = STOP;
}

void michelinDeformationControlInterface::deformationControlGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
	lastOrder = GRASPINGT1S3;
}

void michelinDeformationControlInterface::deformationControlPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
	static int compt = 0;
	//Vector0 : 0.5 * (targetPointLeft to robot point vector)
	static Eigen::Vector3d radiusLeft;
	static Eigen::Vector3d radiusRight;
	//Vector u : rotation axis
	static Eigen::Vector3d uLeft;
	static Eigen::Vector3d uRight;
	Eigen::Vector3d targetPointLeft;
	Eigen::Vector3d targetPointRight;

	static pose previousTargetPointLeft;
	static pose previousTargetPointRight;

	static pose offsetLeft;
	static pose offsetRight;

    static auto lastLoopTime = std::chrono::system_clock::now();
    static auto currentLoopTime = std::chrono::system_clock::now();
	currentLoopTime = std::chrono::system_clock::now();
    std::chrono::duration<float> elapsedTime;
    elapsedTime = currentLoopTime - lastLoopTime;

	
	//If first time : set the target
	if(lastOrder != PERFORMT1S3)
	{
		
		ROS_INFO("deformation control first loop");  
		
		compt = 0;
		std::stringstream ss;
		ss.str(sensorString);
		boost::archive::text_iarchive ia(ss);
		simplestMockupModel sensorData;	
		ia >> sensorData;


		//Offsets
		sensorData.leftArmTargetPose.x += 0.05;//Calibration
		sensorData.leftArmTargetPose.y -= 0.15;
		sensorData.leftArmTargetPose.z += 0.1;

		sensorData.rightArmTargetPose.x += 0.05;
		sensorData.rightArmTargetPose.y += 0.15;
		sensorData.rightArmTargetPose.z += 0.1;



		Eigen::Vector3d tempradiusLeft(0.5*(sensorData.leftArmCurrentPose.x - sensorData.leftArmTargetPose.x), 0.5*(sensorData.leftArmCurrentPose.y - sensorData.leftArmTargetPose.y), 0.5*(sensorData.leftArmCurrentPose.z - sensorData.leftArmTargetPose.z));
		
		Eigen::Vector3d tempradiusRight(0.5*(sensorData.rightArmCurrentPose.x - sensorData.rightArmTargetPose.x), 0.5*(sensorData.rightArmCurrentPose.y - sensorData.rightArmTargetPose.y), 0.5*(sensorData.rightArmCurrentPose.z - sensorData.rightArmTargetPose.z));
		
		radiusLeft = tempradiusLeft;
		radiusRight = tempradiusRight;
		
		uLeft =  radiusLeft.cross(Eigen::Vector3d::UnitZ());
		uLeft.normalize();
		uRight =  radiusRight.cross(Eigen::Vector3d::UnitZ());
		uRight.normalize();
		
		previousTargetPointLeft = sensorData.leftArmCurrentPose;
		previousTargetPointRight = sensorData.rightArmCurrentPose;

		offsetLeft.x = sensorData.leftArmCurrentPose.x - radiusLeft(0);
		offsetLeft.y = sensorData.leftArmCurrentPose.y - radiusLeft(1);
		offsetLeft.z = sensorData.leftArmCurrentPose.z - radiusLeft(2);

		offsetRight.x = sensorData.rightArmCurrentPose.x - radiusRight(0);
		offsetRight.y = sensorData.rightArmCurrentPose.y - radiusRight(1);
		offsetRight.z = sensorData.rightArmCurrentPose.z - radiusRight(2);


		//TEST
		/*for(int burbu = 0; burbu < 250; burbu++)
		{
			targetPointLeft =  Eigen::AngleAxisd(M_PI*burbu/250,uLeft) * radiusLeft;
			//manque une translation pour revenir sur la position du centre du cercle
			targetPointLeft(0) += sensorData.leftArmCurrentPose.x - radiusLeft(0);
			targetPointLeft(1) += sensorData.leftArmCurrentPose.y - radiusLeft(1);
			targetPointLeft(2) += sensorData.leftArmCurrentPose.z - radiusLeft(2);
			ROS_INFO("Target Point Simulated Absolute Position n° %d : x %f, y %f, z %f", burbu, targetPointLeft(0), targetPointLeft(1), targetPointLeft(2));  
	
			pose targetPointLeftPose(targetPointLeft(0), targetPointLeft(1), targetPointLeft(2));
			targetPointLeftPose = UnifiedPositionToEndEffector1(targetPointLeftPose);
			ROS_INFO("Target Point Simulated Relative Position n° %d : x %f, y %f, z %f", burbu, targetPointLeftPose.x, targetPointLeftPose.y, targetPointLeftPose.z);  

		}*/
		//!TEST

	}
	const int maxCompt = 200;
	//if(compt <= maxCompt)
	if(compt <= maxCompt)
	{

		ROS_INFO("deformation control loop %d, elapsed time %f", compt, elapsedTime.count());  

		std::stringstream ss;
		ss.str(sensorString);
		boost::archive::text_iarchive ia(ss);
		simplestMockupModel sensorData;	
		ia >> sensorData;		

		//TEST
		//*
		targetPointLeft =  Eigen::AngleAxisd(M_PI*compt/maxCompt,uLeft) * radiusLeft;
		pose targetPointLeftPose(targetPointLeft(0), targetPointLeft(1), targetPointLeft(2));
		//manque une translation pour revenir sur la position du centre du cercle
		targetPointLeftPose += offsetLeft;

		targetPointRight =  Eigen::AngleAxisd(M_PI*compt/maxCompt,uRight) * radiusRight;
		pose targetPointRightPose(targetPointRight(0), targetPointRight(1), targetPointRight(2));
		//manque une translation pour revenir sur la position du centre du cercle
		targetPointRightPose += offsetRight;//*/

		/*
		pose targetPointLeftPose = offsetLeft;
		targetPointLeftPose.x += (radiusLeft(0) + 0.001*compt);
		targetPointLeftPose.y += radiusLeft(1);
		targetPointLeftPose.z += radiusLeft(2);

		pose targetPointRightPose = offsetRight;
		targetPointRightPose.x += (radiusRight(0) + 0.001*compt);
		targetPointRightPose.y += radiusRight(1);
		targetPointRightPose.z += radiusRight(2);//*/


		//!TEST

		ROS_INFO("Target Point Left Position : x %f, y %f, z %f", targetPointLeftPose.x, targetPointLeftPose.y, targetPointLeftPose.z);  
		ROS_INFO("Target Point Right Position : x %f, y %f, z %f", targetPointRightPose.x, targetPointRightPose.y, targetPointRightPose.z);  

		//We have to convert everything into robotFrame
		

		targetPointLeftPose = UnifiedPositionToEndEffector1(targetPointLeftPose);
		targetPointRightPose =  UnifiedPositionToEndEffector2(targetPointRightPose);

		ROS_INFO("Target Point Left Converted Position : x %f, y %f, z %f", targetPointLeftPose.x, targetPointLeftPose.y, targetPointLeftPose.z);  
		ROS_INFO("Target Point Right Converted Position : x %f, y %f, z %f", targetPointRightPose.x, targetPointRightPose.y, targetPointRightPose.z);  


		ROS_INFO("Position left arm : x %f, y %f, z %f", sensorData.leftArmCurrentPose.x, sensorData.leftArmCurrentPose.y, sensorData.leftArmCurrentPose.z);  
		ROS_INFO("Position right arm : x %f, y %f, z %f", sensorData.rightArmCurrentPose.x, sensorData.rightArmCurrentPose.y, sensorData.rightArmCurrentPose.z);  


		sensorData.leftArmCurrentPose = UnifiedPositionToEndEffector1(sensorData.leftArmCurrentPose);
		sensorData.rightArmCurrentPose = UnifiedPositionToEndEffector2(sensorData.rightArmCurrentPose);

		ROS_INFO("Position left arm Converted : x %f, y %f, z %f", sensorData.leftArmCurrentPose.x, sensorData.leftArmCurrentPose.y, sensorData.leftArmCurrentPose.z);  
		ROS_INFO("Position right arm Converted : x %f, y %f, z %f", sensorData.rightArmCurrentPose.x, sensorData.rightArmCurrentPose.y, sensorData.rightArmCurrentPose.z);  

		//We've got current position, target position, time of a tick.
		//Let's do some middle-school math to find speed.
		//Vitesse = Distance/temps 		
		//current rate is 50Hz

		//TEST
		pose speedToSendLeft;//Type abuse
		pose speedToSendRight;
		//int NopainNogain = elapsedTime.count() * 50;
		
		//*
		speedToSendLeft.x = (targetPointLeftPose.x - sensorData.leftArmCurrentPose.x) * NopainNogain;
		speedToSendLeft.y = (targetPointLeftPose.y - sensorData.leftArmCurrentPose.y) * NopainNogain;
		speedToSendLeft.z = (targetPointLeftPose.z - sensorData.leftArmCurrentPose.z) * NopainNogain;
		//speedToSendLeft.x = 0;
		//speedToSendLeft.y = 0;
		//speedToSendLeft.z = 0;

		speedToSendRight.x = (targetPointRightPose.x - sensorData.rightArmCurrentPose.x) * NopainNogain;
		speedToSendRight.y = (targetPointRightPose.y - sensorData.rightArmCurrentPose.y) * NopainNogain;
		speedToSendRight.z = (targetPointRightPose.z - sensorData.rightArmCurrentPose.z) * NopainNogain;//*/

		//speedToSendRight.x = 0;
		//speedToSendRight.y = 0;
		//speedToSendRight.z = 0;//*/


		/*/
		speedToSendLeft.x = (targetPointLeftPose.x - previousTargetPointLeft.x) * NopainNogain;
		speedToSendLeft.y = (targetPointLeftPose.y - previousTargetPointLeft.y) * NopainNogain;
		speedToSendLeft.z = (targetPointLeftPose.z - previousTargetPointLeft.z) * NopainNogain * 0.2;

		speedToSendRight.x = (targetPointRightPose.x - previousTargetPointRight.x) * NopainNogain;
		speedToSendRight.y = (targetPointRightPose.y - previousTargetPointRight.y) * NopainNogain;
		speedToSendRight.z = (targetPointRightPose.z - previousTargetPointRight.z) * NopainNogain * 0.2;
		//*/		
		// !TEST


		speedToSendLeft.roll = 0;
		speedToSendLeft.pitch = 0;
		speedToSendLeft.yaw = 0;

		speedToSendRight.roll = 0;
		speedToSendRight.pitch = 0;
		speedToSendRight.yaw = 0;


		std::stringstream ssSend;
		boost::archive::text_oarchive oa(ssSend);
		oa << speedToSendLeft;
		oa << speedToSendRight;

		std_msgs::String msg;
		msg.data = ssSend.str();
		pubScoped.pubExternalControl.publish(msg);

		
		previousTargetPointLeft = targetPointLeftPose;
		previousTargetPointRight = targetPointRightPose;

		//TODO : Lancer le externalControl du skilledWorkcell		
		//Pour l'instant, on va se contenter de print
		ROS_INFO("Vitesse demandee gauche : x %f, y %f, z %f", speedToSendLeft.x, speedToSendLeft.y, speedToSendLeft.z);  
		ROS_INFO("Vitesse demandee droite : x %f, y %f, z %f", speedToSendRight.x, speedToSendRight.y, speedToSendRight.z);  

		compt++;
	}
	else
	{

		pose speedToSendLeft;//Type abuse
		speedToSendLeft.x = 0;
		speedToSendLeft.y = 0;
		speedToSendLeft.z = 0;

		pose speedToSendRight;
		speedToSendRight.x = 0;
		speedToSendRight.y = 0;
		speedToSendRight.z = 0;

		speedToSendLeft.roll = 0;
		speedToSendLeft.pitch = 0;
		speedToSendLeft.yaw = 0;

		speedToSendRight.roll = 0;
		speedToSendRight.pitch = 0;
		speedToSendRight.yaw = 0;
		if(compt < (maxCompt + 250))
		{
			speedToSendLeft.roll = 0.1;
			speedToSendLeft.pitch = 0;
			speedToSendLeft.yaw = 0;

			speedToSendRight.roll = 0.1;
			speedToSendRight.pitch = 0;
			speedToSendRight.yaw = 0;
			compt++;

		}
		else
		{ 

			if(compt < (maxCompt + 300))
			{
			
				if(compt == maxCompt + 252)
				{
					ROS_INFO("Trying to open grippers");  

					std_msgs::Bool msgToSend;
					msgToSend.data = true;
					pubScoped.gripperOpen.publish(msgToSend);
				}

				speedToSendLeft.y = -0.1;

				speedToSendRight.y = -0.1;
				compt++;
			} 
			else
			{
				if(compt < (maxCompt + 450))
				{
			

					speedToSendLeft.x = -0.1;

					speedToSendRight.x = 0.1;
					compt++;
				} 

			}
		}	
		//TEST		
		/*
		std::stringstream ss;
		ss.str(sensorString);
		boost::archive::text_iarchive ia(ss);
		simplestMockupModel sensorData;	
		ia >> sensorData;		

		pose targetPointLeftPose = offsetLeft;
		targetPointLeftPose.x += (radiusLeft(0) + 0.001*maxCompt);
		targetPointLeftPose.y += radiusLeft(1);
		targetPointLeftPose.z += radiusLeft(2);

		pose targetPointRightPose = offsetRight;
		targetPointRightPose.x += (radiusRight(0) + 0.001*maxCompt);
		targetPointRightPose.y += radiusRight(1);
		targetPointRightPose.z += radiusRight(2);

		pose speedToSendLeft;//Type abuse
		pose speedToSendRight;

		sensorData.leftArmCurrentPose = UnifiedPositionToEndEffector1(sensorData.leftArmCurrentPose);
		sensorData.rightArmCurrentPose = UnifiedPositionToEndEffector2(sensorData.rightArmCurrentPose);
		targetPointLeftPose = UnifiedPositionToEndEffector1(targetPointLeftPose);
		targetPointRightPose =  UnifiedPositionToEndEffector2(targetPointRightPose);

		speedToSendLeft.x = (targetPointLeftPose.x - sensorData.leftArmCurrentPose.x) * NopainNogain;
		speedToSendLeft.y = (targetPointLeftPose.y - sensorData.leftArmCurrentPose.y) * NopainNogain;
		speedToSendLeft.z = (targetPointLeftPose.z - sensorData.leftArmCurrentPose.z) * NopainNogain;

		speedToSendRight.x = (targetPointRightPose.x - sensorData.rightArmCurrentPose.x) * NopainNogain;
		speedToSendRight.y = (targetPointRightPose.y - sensorData.rightArmCurrentPose.y) * NopainNogain;
		speedToSendRight.z = (targetPointRightPose.z - sensorData.rightArmCurrentPose.z) * NopainNogain;//*/
		//!TEST


		ROS_INFO("Vitesse demandee : x %f, y %f, z %f", speedToSendRight.x, speedToSendRight.y, speedToSendRight.z);  

		std::stringstream ssSend;
		boost::archive::text_oarchive oa(ssSend);
		oa << speedToSendLeft;
		oa << speedToSendRight;

		std_msgs::String msg;
		msg.data = ssSend.str();
		pubScoped.pubExternalControl.publish(msg);

	
		if(compt >= (maxCompt + 450))
		{
			notifySupervisorTaskDone();
		}	
	}
		/*std::stringstream ss;
		ss.str(sensorString);
		boost::archive::text_iarchive ia(ss);
			

		ia >> tempPose;
		ia >> tempPose2;*/								

		/*TEST
		pose tempPose(-0.3, -0.15, 0);
		pose tempPose2(-0.3, +0.20, 0);;
	
		pose leftArmPosition(0, -0.20, -0.05);
		pose rightArmPosition(0, +0.20, -0.05); //TODO : put them in perception. Unserialize them.

		//Vector0 : 0.5 * (targetPointLeft to robot point vector)
		Eigen::Vector3d radiusLeft(0.5*(leftArmPosition.x - tempPose.x), 0.5*(leftArmPosition.y - tempPose.y), 0.5*(leftArmPosition.z - tempPose.z));
		
		//Vector u : rotation axis
		Eigen::Vector3d u =  radiusLeft.cross(Eigen::Vector3d::UnitZ());
		u.normalize();

		Eigen::Vector3d targetPointLeft;

		ROS_INFO("Begin");  
		for(int i = 0; i < 12; i++)
		{		
			targetPointLeft =  Eigen::AngleAxisd(M_PI*i/12,u) * radiusLeft;
			//manque une translation pour revenir sur la position du centre du cercle
			targetPointLeft(0) += leftArmPosition.x - radiusLeft(0);
			targetPointLeft(1) += leftArmPosition.y - radiusLeft(1);
			targetPointLeft(2) += leftArmPosition.z - radiusLeft(2);
			
			ROS_INFO("Absolute pose : x %f, y %f, z %f", targetPointLeft(0), targetPointLeft(1), targetPointLeft(2));  
		}
		ROS_INFO("End");  
		*/ //!TEST
	lastLoopTime = currentLoopTime;
	lastOrder = PERFORMT1S3;
}	

 
static void notifySupervisorTaskDone(void)
{
	ROS_INFO("Notifying supervisor we're done");

	std_msgs::Bool msgToSend;
	msgToSend.data = true;
	pubScoped.pubDeformationControlStatus.publish(msgToSend);
}


//std::vector<pose> specificDeformationControl_multiLayerAssembly_getTargetPose(std::string paramStr)
//{

	/*static bool isFirstTime = true;
	zmq::context_t context(1);
    static zmq::socket_t publisher(context, ZMQ_PUB);
	
	if(isFirstTime)
	{
		isFirstTime = false;
		publisher.bind("tcp://*:5564");		
	}*/

/*	std::string str = paramStr;		
	std::stringstream ss(str);
	boost::archive::text_iarchive ia(ss);
	//ss.str(str);
	
	//pose tarpos;
	float errorMesured;	
	float cumulativeError = 0;
	std::vector<pose> retval;
	
	ia >> errorMesured;	
	
	float Kp = -0.04;
	float Ki = -0.001;
	float Kd = -0.12;	

	lastError[currentErrorIndex] = errorMesured;
	
	for(int i = 0; i < k_Horizon; i++)
	{
		cumulativeError += lastError[i];
	}
	cumulativeError = cumulativeError/ k_Horizon;
	
	
	float correction = 	-(Kp * errorMesured + Ki * cumulativeError + Kd*(errorMesured - lastTimeError));		
	
	if((errorMesured > 0.03) || (errorMesured < -0.03))
	{

		targetPoseArm1.x += correction;
		std::cout << "Correction : " << (Kp * errorMesured + Ki * cumulativeError + Kd*(errorMesured - lastTimeError)) << std::endl;
			
	}
	else
	{
		correction = 0;
	}

	//s_send (publisher, "test alc");	
	std::ofstream outfile;
	outfile.open("test.txt", std::ios_base::app);
	outfile << correction;
	outfile << std::endl;
	outfile.close();	

	currentErrorIndex++;
	currentErrorIndex = currentErrorIndex % k_Horizon;
	lastTimeError = errorMesured;*/	

	/*targetPoseArm1.x += tarpos.x;
	targetPoseArm1.y += tarpos.y;
	targetPoseArm1.z += tarpos.z;
	targetPoseArm1.roll += tarpos.roll;
	targetPoseArm1.pitch += tarpos.pitch;
	targetPoseArm1.yaw += tarpos.yaw;*/

	//std::cout << tarpos.x << " " << tarpos.y << " " << tarpos.z << " " << std::endl;	

	/*pose returnPose = targetPoseArm1;	
	retval.clear();	
	retval.push_back(returnPose);
	return retval;*/

	/*ia >> tarpos;	

	targetPoseArm2.x = tarpos.posx;
	targetPoseArm2.y = tarpos.posy;
	retval.push_back(targetPoseArm2);

	return retval;*/
//}

void specificDeformationControlRosInit(void)
{
	ros::NodeHandle nh;
	pubScoped.pubExternalControl = nh.advertise<std_msgs::String>("deformationControl", 1);
	pubScoped.pubDeformationControlStatus = nh.advertise<std_msgs::Bool>("deformationControlStepDone", 1);
	
	pubScoped.gripperOpen = nh.advertise<std_msgs::Bool>("gripperOpen", 1);
	pubScoped.gripperClose = nh.advertise<std_msgs::Bool>("gripperClose", 1);

	return;
}

void specificDeformationControl_setSensor(std::string param)
{
	ROS_INFO("sensor Callback");  
	sensorString = param;
}

