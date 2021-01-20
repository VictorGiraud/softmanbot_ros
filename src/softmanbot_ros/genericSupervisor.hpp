#ifndef GENERIC_SUPERVISOR_HPP
#define GENERIC_SUPERVISOR_HPP

#include <ros/ros.h>

//This class is the template for the supervisory interface. You will have to create a specific interface inheriting from this one.
//Name of the skills are very generic. This way, if you want to reuse for anything, you will have access to 3 tasks with at least 3 skills each, not tied to original softmanbot application.

class supervisoryInterface
{
public:
	supervisoryInterface(void) {};
	~supervisoryInterface(void) {};

	virtual void supervisoryInit(void) {throw std::string("exception : Not implemented");};
	virtual void supervisoryIdle(void) {throw std::string("exception : Not implemented");};
	virtual void supervisoryStop(void) {throw std::string("exception : Not implemented");};

	virtual void supervisoryGraspingT1S1(void) {throw std::string("exception : Not implemented");}; //Precise positioning of a 2D layer with minimum deformation
	virtual void supervisoryPerformT1S1(void) {throw std::string("exception : Not implemented");};	
	virtual void supervisoryGraspingT1S2(void) {throw std::string("exception : Not implemented");}; //Wrapping a 2D layer around a 3D support
	virtual void supervisoryPerformT1S2(void) {throw std::string("exception : Not implemented");};
	virtual void supervisoryGraspingT1S3(void) {throw std::string("exception : Not implemented");}; //Precise joining of 2 layers
	virtual void supervisoryPerformT1S3(void) {throw std::string("exception : Not implemented");};

	virtual void supervisoryGraspingT2S1(void) {throw std::string("exception : Not implemented");}; //Optimize grasping configuration for pulling motion, avoiding sliding
	virtual void supervisoryPerformT2S1(void) {throw std::string("exception : Not implemented");};	
	virtual void supervisoryGraspingT2S2(void) {throw std::string("exception : Not implemented");}; //Detaching operations for introducing air in the mold, avoiding sliding
	virtual void supervisoryPerformT2S2(void) {throw std::string("exception : Not implemented");};  
	virtual void supervisoryGraspingT2S3(void) {throw std::string("exception : Not implemented");}; //Pulling while minimizing deformation and avoiding sliding
	virtual void supervisoryPerformT2S3(void) {throw std::string("exception : Not implemented");};
	virtual void supervisoryGraspingT2S4(void) {throw std::string("exception : Not implemented");}; //Re-grasping if failure (sliding) or not complete extraction
	virtual void supervisoryPerformT2S4(void) {throw std::string("exception : Not implemented");};

	virtual void supervisoryGraspingT3S1(void) {throw std::string("exception : Not implemented");}; // Optimize grasping configuration for insertion motion, avoiding sliding
	virtual void supervisoryPerformT3S1(void) {throw std::string("exception : Not implemented");};	
	virtual void supervisoryGraspingT3S2(void) {throw std::string("exception : Not implemented");}; // Insertion of one soft object inside another one
	virtual void supervisoryPerformT3S2(void) {throw std::string("exception : Not implemented");};
	virtual void supervisoryGraspingT3S3(void) {throw std::string("exception : Not implemented");}; // Re-grasping if failure (sliding) or not complete insertion
	virtual void supervisoryPerformT3S3(void) {throw std::string("exception : Not implemented");};
};

#endif //GENERIC_SUPERVISOR_HPP
