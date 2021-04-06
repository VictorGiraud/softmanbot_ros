#include "specificPerception.hpp"
#include <Python.h>

#include "zhelpers.hpp"

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <sstream>

#include "genericLogic.hpp"

#include "iostream"
#include "sstream"

#include "vision_reza.hpp"

#include "positionConverter.hpp"
#include <geometry_msgs/Pose.h>


void testPython(void);
void testZmq(void);

PyObject *pLogixName;
PyObject *pLogixModule;

static softmanbotState lastOrder = IDLE;//I want to do init interface action once. Thus, I have to keep memory of last action treated 

static void notifySupervisorTaskDone(void);

static void positionLeftCallback(const geometry_msgs::Pose::ConstPtr& msg);
static void positionRightCallback(const geometry_msgs::Pose::ConstPtr& msg);



class pubHandler
{
public:
	ros::Publisher pubPerceptionStatus;
	ros::Publisher specificSensorPub;
};

pubHandler pubScoped;

class subHandler
{
public:
	ros::Subscriber subLeftArmPositionCartesianController;
	ros::Subscriber subRightArmPositionCartesianController;
	ros::Subscriber subLeftArmPositionSpeedController;
	ros::Subscriber subRightArmPositionSpeedController;
};

subHandler subScoped;
simplestMockupModel sensorDataScoped;


static void notifySupervisorTaskDone(void)
{
	ROS_INFO("Notifying supervisor we're done");

	std_msgs::Bool msgToSend;
	msgToSend.data = true;
	pubScoped.pubPerceptionStatus.publish(msgToSend);
}


perceptionInterface& getPerceptionInterface(void)
{
	static michelinPerceptionInterface retval;
	return retval;
}

void michelinPerceptionInterface::perceptionInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
	//Let's put Reza's perception here
	if(lastOrder != INIT)
	{	
		vector<double> left_point;
		vector<double> right_point;
		DoubleVector2D final_transformation;
		reza_vision(final_transformation, left_point, right_point, false);//true for enter to confirm mode

		ROS_INFO("final_transformation : x %f, y %f, z %f, w %f", final_transformation[0][0], final_transformation[0][1], final_transformation[0][2], final_transformation[0][3]); 
		ROS_INFO("final_transformation : x %f, y %f, z %f, w %f", final_transformation[1][0], final_transformation[1][1], final_transformation[1][2], final_transformation[1][3]); 
		ROS_INFO("final_transformation : x %f, y %f, z %f, w %f", final_transformation[2][0], final_transformation[2][1], final_transformation[2][2], final_transformation[2][3]); 
		ROS_INFO("final_transformation : x %f, y %f, z %f, w %f", final_transformation[3][0], final_transformation[3][1], final_transformation[3][2], final_transformation[3][3]); 


		ROS_INFO("left pose : x %f, y %f, z %f", left_point[0], left_point[1], left_point[2]); 
		ROS_INFO("right pose : x %f, y %f, z %f", right_point[0], right_point[1], right_point[2]); 
		pose cameraLeftPose (left_point[0], left_point[1], left_point[2]);
		pose cameraRightPose(right_point[0], right_point[1], right_point[2]);

		sensorDataScoped.leftArmTargetPose = CameraToUnifiedPosition(cameraLeftPose);
		sensorDataScoped.rightArmTargetPose = CameraToUnifiedPosition(cameraRightPose);	
		
		
		ROS_INFO("Unified left pose : x %f, y %f, z %f", sensorDataScoped.leftArmTargetPose.x, sensorDataScoped.leftArmTargetPose.y, sensorDataScoped.leftArmTargetPose.z); 
		ROS_INFO("Unified right pose : x %f, y %f, z %f", sensorDataScoped.rightArmTargetPose.x, sensorDataScoped.rightArmTargetPose.y, sensorDataScoped.rightArmTargetPose.z); 

		ROS_INFO("End of perception init");
		notifySupervisorTaskDone();			
	}
	lastOrder = INIT;
}

void michelinPerceptionInterface::perceptionIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
	lastOrder = IDLE;
}

void michelinPerceptionInterface::perceptionStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
	lastOrder = STOP;
}

void michelinPerceptionInterface::perceptionGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
	lastOrder = GRASPINGT1S3;

	std::stringstream ssend;
	boost::archive::text_oarchive oa(ssend);
	oa << sensorDataScoped;

	std_msgs::String msg;
	msg.data = ssend.str();
	pubScoped.specificSensorPub.publish(msg);
	
}

void michelinPerceptionInterface::perceptionPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
	lastOrder = PERFORMT1S3;

	std::stringstream ssend;
	boost::archive::text_oarchive oa(ssend);
	oa << sensorDataScoped;

	std_msgs::String msg;
	msg.data = ssend.str();
	pubScoped.specificSensorPub.publish(msg);
	
}	


void testPython(void)//TODO : create an object 
{

	Py_Initialize();
	
	pLogixName = PyUnicode_FromString("plcComm");	
	pLogixModule = PyImport_Import(pLogixName); //For this to work, i added plcComm into pythonpath in file ~/.bashrc
	
	if(pLogixModule != NULL)
	{
		std::cout << "moduleImported" << std::endl;
		/*PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "writeTag");
		PyObject *py_args = PyTuple_New(2);
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("ZZ_VIC"));				
		PyTuple_SetItem(py_args, 1, PyLong_FromLong(7));	
		PyObject *pValue = PyObject_CallObject(pFunc, py_args);//NULL instead of py_args if no arguments*/
		
		PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "readTag");
		PyObject *py_args = PyTuple_New(1);
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("ZZ_VIC"));				
		PyObject *pValue = PyObject_CallObject(pFunc, py_args);//NULL instead of py_args if no arguments*/
		PyObject *readValue = PyObject_GetAttrString(pValue, "Value");		
	
		std::cout << "Read value " << PyLong_AsLong(readValue) << std::endl;
		//PyObject *py_args = PyTuple_New(2);				
		//PyTuple_SetItem(py_args, 0, PyLong_FromLong(6));//PyUnicode_FromString; PyFloat_AsDouble...
		//PyTuple_SetItem(py_args, 1, PyLong_FromLong(7));	
		//std::cout << "6 x 7 :" << PyLong_AsLong(pValue) << std::endl;
	}
	else
	{
		std::cout << "pModule nullptr" << std::endl;
		//TODO : send an error of fail initialization	
	}
	Py_Finalize();
	return;
	/*PyRun_SimpleString("import sys");
	PyRun_SimpleString("print(\"Python talking to snakes.\")");*/
	
	/*PyRun_SimpleString("import sys");	
	PyRun_SimpleString("from pylogix import PLC");
	PyRun_SimpleString("with PLC() as comm:");
	PyRun_SimpleString("comm.IPAddress = '192.168.1.100'");
    PyRun_SimpleString("ret = comm.Read('Program:PC_CommonPart.R010_ConsVitesse')");//R078_URD.UR_Order_done
    PyRun_SimpleString("print(ret.TagName, ret.Value, ret.Status)");*/

	/*if(pModule != NULL)
	{
		PyObject *pFunc = PyObject_GetAttrString(pModule, "getInteger");
		PyObject *py_args = PyTuple_New(2);
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("6"));
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("7"));	
		PyObject *pValue = PyObject_CallObject(pFunc, py_args);
		std::cout << "6 x 7 :" << PyLong_AsLong(pValue) << std::endl;
	}
	*/

	/*
		PyObject *pFunc = PyObject_GetAttrString(pModule, "getInteger");
		std::cout << "AttrString gotten" << std::endl;
		PyObject *pValue = PyObject_CallObject(pFunc, NULL);
		std::cout << "return value :" << PyLong_AsLong(pValue) << std::endl;
	*/
}

void testZmq(void)
{
	zmq::context_t context(1);
    zmq::socket_t subscriber (context, ZMQ_SUB);

    std::cout << "trying to connect :" << std::endl;

    subscriber.connect("tcp://192.168.1.120:5563");
    std::cout << "Connection OK :" << std::endl;

    subscriber.setsockopt( ZMQ_SUBSCRIBE, "", 0);

	while(1)
	{
		//  Read message contents
        std::string contents = s_recv (subscriber);		

		std::stringstream ss(contents);
		/*pose tarpos;
		ss >> tarpos.x;
		ss >> tarpos.y;
		ss >> tarpos.z;
		ss >> tarpos.roll;
		ss >> tarpos.pitch;
		ss >> tarpos.yaw;

		std::cout << "received this :" << tarpos.x << " " << tarpos.y << " " << tarpos.z << " " << tarpos.roll << " "<< tarpos.pitch << " " << tarpos.yaw << " " << std::endl; 
	  	*/		
		float errorReceived;
		ss >> errorReceived;

		std::stringstream ssend;
		boost::archive::text_oarchive oa(ssend);
		oa << errorReceived;
		
		std::cout << "received sthg :" << errorReceived << std::endl;

		std_msgs::String msg;
		msg.data = ssend.str();
		pubScoped.specificSensorPub.publish(msg);
	}


    return;
}

void specificPerceptionRosInit(void)
{
	ros::NodeHandle nh;
	pubScoped.pubPerceptionStatus = nh.advertise<std_msgs::Bool>("perceptionStepDone", 1);
	pubScoped.specificSensorPub  = nh.advertise<std_msgs::String>("specific_perception", 1);


	subScoped.subLeftArmPositionCartesianController = nh.subscribe("/Arm1/cartesian_velocity_control/current_x", 50, positionLeftCallback);
	subScoped.subRightArmPositionCartesianController = nh.subscribe("/Arm2/cartesian_velocity_control/current_x", 50, positionRightCallback);
	subScoped.subLeftArmPositionSpeedController = nh.subscribe("/Arm1/vel_based_cartesian_velocity_control/current_x", 50, positionLeftCallback);
	subScoped.subRightArmPositionSpeedController = nh.subscribe("/Arm2/vel_based_cartesian_velocity_control/current_x", 50, positionRightCallback);
	
	return;
}


static void positionLeftCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	//Getting quaternion	
	Eigen::Quaternion<double> qAnon(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	//Convert to rpy
	pose anonPose = quaternionToRPY(qAnon);
	anonPose.x = msg->position.x;
	anonPose.y = msg->position.y;
	anonPose.z = msg->position.z;

	ROS_INFO("relative left pose : x %f, y %f, z %f", anonPose.x, anonPose.y, anonPose.z); 
	sensorDataScoped.leftArmCurrentPose = EndEffector1ToUnifiedPosition(anonPose);

	ROS_INFO("Unified left pose : x %f, y %f, z %f", sensorDataScoped.leftArmCurrentPose.x, sensorDataScoped.leftArmCurrentPose.y, sensorDataScoped.leftArmCurrentPose.z); 
}


static void positionRightCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	//Getting quaternion	
	Eigen::Quaternion<double> qAnon(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	//Convert to rpy
	pose anonPose = quaternionToRPY(qAnon);
	anonPose.x = msg->position.x;
	anonPose.y = msg->position.y;
	anonPose.z = msg->position.z;

	ROS_INFO("relative right pose : x %f, y %f, z %f", anonPose.x, anonPose.y, anonPose.z); 
	
	sensorDataScoped.rightArmCurrentPose = EndEffector2ToUnifiedPosition(anonPose);

	ROS_INFO("Unified right pose : x %f, y %f, z %f", sensorDataScoped.rightArmCurrentPose.x, sensorDataScoped.rightArmCurrentPose.y, sensorDataScoped.rightArmCurrentPose.z); 

}



/*
    while (1) {

        //  Read envelope with address
        //std::string address = s_recv (subscriber);
        //  Read message contents
        std::string contents = s_recv (subscriber);
        
		std::stringstream ss(contents);
		
		// Running loop till the end of the stream
		std::string temp; 
		float found;
		float sum = 0; 
		while (!ss.eof()) { 
	  
		    // extracting word by word from stream
		    ss >> temp; 
	  
		    // Checking the given word is integer or not
		    if (std::stringstream(temp) >> found) 
		        std::cout << "found this :" << found << " "; 
	  			sum += found;
		    // To save from space at the end of string 
		    temp = ""; 
		} 

        std::cout << "original contents :" << contents << " sum  : " << sum << std::endl;
    }

*/

/*
from pylogix import PLC
with PLC() as comm:
    comm.IPAddress = '192.168.1.100'
    ret = comm.Read('Program:PC_CommonPart.R010_ConsVitesse')
    print(ret.TagName, ret.Value, ret.Status)*/
