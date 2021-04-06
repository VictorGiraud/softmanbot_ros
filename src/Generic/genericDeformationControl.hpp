#ifndef GENERIC_DEFORMATION_CONTROL_HPP
#define GENERIC_DEFORMATION_CONTROL_HPP

#include <ros/ros.h>

//This class is the template for the deformationControl interface. You will have to create a specific interface inheriting from this one.
//Name of the skills are very generic. This way, if you want to reuse for anything, you will have access to 3 tasks with at least 3 skills each, not tied to original softmanbot application.

class deformationControlInterface
{
public:
	deformationControlInterface(void) {};
	~deformationControlInterface(void) {};

	virtual void deformationControlInit(void) {throw std::string("exception : Not implemented");};
	virtual void deformationControlIdle(void) {throw std::string("exception : Not implemented");};
	virtual void deformationControlStop(void) {throw std::string("exception : Not implemented");};

	virtual void deformationControlGraspingT1S1(void) {throw std::string("exception : Not implemented");}; //Precise positioning of a 2D layer with minimum deformation
	virtual void deformationControlPerformT1S1(void) {throw std::string("exception : Not implemented");};	
	virtual void deformationControlGraspingT1S2(void) {throw std::string("exception : Not implemented");}; //Wrapping a 2D layer around a 3D support
	virtual void deformationControlPerformT1S2(void) {throw std::string("exception : Not implemented");};
	virtual void deformationControlGraspingT1S3(void) {throw std::string("exception : Not implemented");}; //Precise joining of 2 layers
	virtual void deformationControlPerformT1S3(void) {throw std::string("exception : Not implemented");};

	virtual void deformationControlGraspingT2S1(void) {throw std::string("exception : Not implemented");}; //Optimize grasping configuration for pulling motion, avoiding sliding
	virtual void deformationControlPerformT2S1(void) {throw std::string("exception : Not implemented");};	
	virtual void deformationControlGraspingT2S2(void) {throw std::string("exception : Not implemented");}; //Detaching operations for introducing air in the mold, avoiding sliding
	virtual void deformationControlPerformT2S2(void) {throw std::string("exception : Not implemented");};  
	virtual void deformationControlGraspingT2S3(void) {throw std::string("exception : Not implemented");}; //Pulling while minimizing deformation and avoiding sliding
	virtual void deformationControlPerformT2S3(void) {throw std::string("exception : Not implemented");};
	virtual void deformationControlGraspingT2S4(void) {throw std::string("exception : Not implemented");}; //Re-grasping if failure (sliding) or not complete extraction
	virtual void deformationControlPerformT2S4(void) {throw std::string("exception : Not implemented");};

	virtual void deformationControlGraspingT3S1(void) {throw std::string("exception : Not implemented");}; // Optimize grasping configuration for insertion motion, avoiding sliding
	virtual void deformationControlPerformT3S1(void) {throw std::string("exception : Not implemented");};	
	virtual void deformationControlGraspingT3S2(void) {throw std::string("exception : Not implemented");}; // Insertion of one soft object inside another one
	virtual void deformationControlPerformT3S2(void) {throw std::string("exception : Not implemented");};
	virtual void deformationControlGraspingT3S3(void) {throw std::string("exception : Not implemented");}; // Re-grasping if failure (sliding) or not complete insertion
	virtual void deformationControlPerformT3S3(void) {throw std::string("exception : Not implemented");};
};

#endif //GENERIC_DEFORMATION_CONTROL_HPP
