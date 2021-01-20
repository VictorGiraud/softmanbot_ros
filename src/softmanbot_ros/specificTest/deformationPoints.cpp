#include <ros/ros.h>
#include "deformationPoints.hpp"

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
	srand(time(NULL));
	
	//Create a deformablePoints Object
	Point tl(0, 10);
	Point tr(10, 10);
	Point bl(0, 0);
	Point br(10, 0);
	
	//create a publisher object
	ros::Publisher pub = nh.advertise<std_msgs::String>("sensor_deformation", 1000);

	//Loop at 2Hz until node shut down
	ros::Rate rate(2);	
	while(ros::ok())
	{
		std::stringstream ss;
    	boost::archive::text_oarchive oa(ss);
		    	
		int randomnumberX = (rand() % 5) - 2;	
		int randomnumberY = (rand() % 5) - 2;	
		tr.move(randomnumberX, randomnumberY);

		DeformationPoints myObject(tl, tr, bl, br); 	
		deformationType defType = POINTS;
		oa << defType;	
		oa << myObject;
	
		std_msgs::String msg;
		msg.data = ss.str();
		pub.publish(msg);
		rate.sleep();
	}
}

