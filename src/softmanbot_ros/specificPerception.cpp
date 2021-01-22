#include "specificPerception.hpp"

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <sstream>

#include "genericLogic.hpp"

#include "iostream"
#include "sstream"

static ros::Publisher specificSensorPub;


/*void specificPerception_init(void)
{
	//Initialising the publisher object
    ros::NodeHandle nh;
	specificSensorPub = nh.advertise<std_msgs::String>("specific_perception", 1000);

	testZmq();//testing for communication with Nicolas
	//testPython();
}*/

perceptionInterface& getPerceptionInterface(void)
{
	static michelinPerceptionInterface retval;
	return retval;
}

void michelinPerceptionInterface::perceptionInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
}

void michelinPerceptionInterface::perceptionIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
}

void michelinPerceptionInterface::perceptionStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
}

void michelinPerceptionInterface::perceptionGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
}

void michelinPerceptionInterface::perceptionPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
}	


/*
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
		/*float errorReceived;
		ss >> errorReceived;

		std::stringstream ssend;
		boost::archive::text_oarchive oa(ssend);
		oa << errorReceived;
		
		std::cout << "received sthg :" << errorReceived << std::endl;

		std_msgs::String msg;
		msg.data = ssend.str();
		specificSensorPub.publish(msg);
	}


    return;
}*/

void specificPerceptionRosInit(void)
{
	return;
}


