#include <ros/ros.h>
#include "deformationForce.hpp"

#include "std_msgs/String.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <sstream>
#include <time.h>
#include <stdlib.h>


int main (int argc, char **argv)
{
	//Initialize the ROS system, become a node
	ros::init(argc, argv, "deformationForce");
	ros::NodeHandle nh;

	//seed random	
	srand(time(NULL));
	
	//create a publisher object
	ros::Publisher pub = nh.advertise<std_msgs::String>("sensor_deformation", 1000);

	//Loop at 2Hz until node shut down
	ros::Rate rate(2);	
	while(ros::ok())
	{
		std::stringstream ss;
    	boost::archive::text_oarchive oa(ss);
		    	
		//Create a deformableForce Object
		int randomnumber = (rand() % 3) - 1;	
		static float squish = 1;
		squish += 0.1*randomnumber;
		if(squish <= 0)
		{
			squish = 0.1;
		}
		if(squish >= 2)
		{
			squish = 1.9;	
		}
		DeformationForce myObject(squish); 
		deformationType defType = FORCE;
		oa << defType;	
		oa << myObject;

		std_msgs::String msg;
		msg.data = ss.str();
		pub.publish(msg);
		rate.sleep();
	}
}
