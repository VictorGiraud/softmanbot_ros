#include "specificDeformationControl.hpp"

#include <boost/archive/text_iarchive.hpp>
#include <sstream>
#include <fstream>

#include "deformationMSD.hpp"

#include <ros/ros.h>

std::vector<pose> specificDeformationControl_multiLayerAssembly_getTargetPose(std::string str)
{

	std::stringstream ss(str);
	pose targetPoseArm1, targetPoseArm2;
	DeformationMSD model;
	targetPosition tarpos;
	std::vector<pose> retval;
	

	ss.str(str);
	boost::archive::text_iarchive ia(ss);

	ia >> model;
	ia >> tarpos;	

	targetPoseArm1.x = tarpos.posx;
	targetPoseArm1.y = tarpos.posy;
	retval.push_back(targetPoseArm1);
	
	ia >> tarpos;	

	targetPoseArm2.x = tarpos.posx;
	targetPoseArm2.y = tarpos.posy;
	retval.push_back(targetPoseArm2);

	return retval;
}
