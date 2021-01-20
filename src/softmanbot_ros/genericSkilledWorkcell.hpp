#ifndef GENERIC_SKILLEDWORKCELL_HPP
#define GENERIC_SKILLEDWORKCELL_HPP

#include <ros/ros.h>

//This class is the template for the supervisory interface. You will have to create a specific interface inheriting from this one.
//Name of the skills are very generic. This way, if you want to reuse for anything, you will have access to 3 tasks with at least 3 skills each, not tied to original softmanbot application.

class skilledWorkcellInterface
{
public:
	skilledWorkcellInterface(void) {};
	~skilledWorkcellInterface(void) {};

	virtual void skilledWorkcellInit(void) {throw std::string("exception : Not implemented");};
	virtual void skilledWorkcellIdle(void) {throw std::string("exception : Not implemented");};
	virtual void skilledWorkcellStop(void) {throw std::string("exception : Not implemented");};

	virtual void skilledWorkcellGraspingT1S1(void) {throw std::string("exception : Not implemented");}; //Precise positioning of a 2D layer with minimum deformation
	virtual void skilledWorkcellPerformT1S1(void) {throw std::string("exception : Not implemented");};	
	virtual void skilledWorkcellGraspingT1S2(void) {throw std::string("exception : Not implemented");}; //Wrapping a 2D layer around a 3D support
	virtual void skilledWorkcellPerformT1S2(void) {throw std::string("exception : Not implemented");};
	virtual void skilledWorkcellGraspingT1S3(void) {throw std::string("exception : Not implemented");}; //Precise joining of 2 layers
	virtual void skilledWorkcellPerformT1S3(void) {throw std::string("exception : Not implemented");};

	virtual void skilledWorkcellGraspingT2S1(void) {throw std::string("exception : Not implemented");}; //Optimize grasping configuration for pulling motion, avoiding sliding
	virtual void skilledWorkcellPerformT2S1(void) {throw std::string("exception : Not implemented");};	
	virtual void skilledWorkcellGraspingT2S2(void) {throw std::string("exception : Not implemented");}; //Detaching operations for introducing air in the mold, avoiding sliding
	virtual void skilledWorkcellPerformT2S2(void) {throw std::string("exception : Not implemented");};  
	virtual void skilledWorkcellGraspingT2S3(void) {throw std::string("exception : Not implemented");}; //Pulling while minimizing deformation and avoiding sliding
	virtual void skilledWorkcellPerformT2S3(void) {throw std::string("exception : Not implemented");};
	virtual void skilledWorkcellGraspingT2S4(void) {throw std::string("exception : Not implemented");}; //Re-grasping if failure (sliding) or not complete extraction
	virtual void skilledWorkcellPerformT2S4(void) {throw std::string("exception : Not implemented");};

	virtual void skilledWorkcellGraspingT3S1(void) {throw std::string("exception : Not implemented");}; // Optimize grasping configuration for insertion motion, avoiding sliding
	virtual void skilledWorkcellPerformT3S1(void) {throw std::string("exception : Not implemented");};	
	virtual void skilledWorkcellGraspingT3S2(void) {throw std::string("exception : Not implemented");}; // Insertion of one soft object inside another one
	virtual void skilledWorkcellPerformT3S2(void) {throw std::string("exception : Not implemented");};
	virtual void skilledWorkcellGraspingT3S3(void) {throw std::string("exception : Not implemented");}; // Re-grasping if failure (sliding) or not complete insertion
	virtual void skilledWorkcellPerformT3S3(void) {throw std::string("exception : Not implemented");};
};

#endif
