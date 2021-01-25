#ifndef GENERIC_PERCEPTION_HPP
#define GENERIC_PERCEPTION_HPP

#include <ros/ros.h>

//This class is the template for the perception interface. You will have to create a specific interface inheriting from this one.
//Name of the skills are very generic. This way, if you want to reuse for anything, you will have access to 3 tasks with at least 3 skills each, not tied to original softmanbot application.

class perceptionInterface
{
public:
	perceptionInterface(void) {};
	~perceptionInterface(void) {};

	virtual void perceptionInit(void) {throw std::string("exception : Not implemented");};
	virtual void perceptionIdle(void) {throw std::string("exception : Not implemented");};
	virtual void perceptionStop(void) {throw std::string("exception : Not implemented");};

	virtual void perceptionGraspingT1S1(void) {throw std::string("exception : Not implemented");}; //Precise positioning of a 2D layer with minimum deformation
	virtual void perceptionPerformT1S1(void) {throw std::string("exception : Not implemented");};	
	virtual void perceptionGraspingT1S2(void) {throw std::string("exception : Not implemented");}; //Wrapping a 2D layer around a 3D support
	virtual void perceptionPerformT1S2(void) {throw std::string("exception : Not implemented");};
	virtual void perceptionGraspingT1S3(void) {throw std::string("exception : Not implemented");}; //Precise joining of 2 layers
	virtual void perceptionPerformT1S3(void) {throw std::string("exception : Not implemented");};

	virtual void perceptionGraspingT2S1(void) {throw std::string("exception : Not implemented");}; //Optimize grasping configuration for pulling motion, avoiding sliding
	virtual void perceptionPerformT2S1(void) {throw std::string("exception : Not implemented");};	
	virtual void perceptionGraspingT2S2(void) {throw std::string("exception : Not implemented");}; //Detaching operations for introducing air in the mold, avoiding sliding
	virtual void perceptionPerformT2S2(void) {throw std::string("exception : Not implemented");};  
	virtual void perceptionGraspingT2S3(void) {throw std::string("exception : Not implemented");}; //Pulling while minimizing deformation and avoiding sliding
	virtual void perceptionPerformT2S3(void) {throw std::string("exception : Not implemented");};
	virtual void perceptionGraspingT2S4(void) {throw std::string("exception : Not implemented");}; //Re-grasping if failure (sliding) or not complete extraction
	virtual void perceptionPerformT2S4(void) {throw std::string("exception : Not implemented");};

	virtual void perceptionGraspingT3S1(void) {throw std::string("exception : Not implemented");}; // Optimize grasping configuration for insertion motion, avoiding sliding
	virtual void perceptionPerformT3S1(void) {throw std::string("exception : Not implemented");};	
	virtual void perceptionGraspingT3S2(void) {throw std::string("exception : Not implemented");}; // Insertion of one soft object inside another one
	virtual void perceptionPerformT3S2(void) {throw std::string("exception : Not implemented");};
	virtual void perceptionGraspingT3S3(void) {throw std::string("exception : Not implemented");}; // Re-grasping if failure (sliding) or not complete insertion
	virtual void perceptionPerformT3S3(void) {throw std::string("exception : Not implemented");};
};

#endif //GENERIC_PERCEPTION_HPP
