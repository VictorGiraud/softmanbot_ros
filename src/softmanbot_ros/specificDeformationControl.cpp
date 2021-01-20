#include "specificDeformationControl.hpp"
#include <boost/archive/text_iarchive.hpp>	

#include <sstream>
#include <fstream>

#include "genericLogic.hpp"
#include <ros/ros.h>

#include "zhelpers.hpp"


static pose targetPoseArm1;

constexpr uint8_t PID_HORIZON = 10;

static uint8_t currentErrorIndex;

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

	//Check if first time called. If it is, reset all PID
	//Do work periodically
	//What we are doing : We have from perception where we are (robot position) and where we want to go (edge to solder position)
	
	//this function will take track of the skill time

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
	//Here is where we'll declare our Ros publisher/subscribers that doesn't fit the global architecture
	return;
}
