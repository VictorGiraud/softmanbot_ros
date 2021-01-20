#ifndef SPECIFIC_DEFORMATION_CONTROL_HPP
#define SPECIFIC_DEFORMATION_CONTROL_HPP

#include "genericDeformationControl.hpp"

//All specific should get this prototype needed by generic
deformationControlInterface& 	getDeformationControlInterface(void);
void							specificDeformationControlRosInit(void);

//Custom declaration of inherited interface
class michelinDeformationControlInterface: public deformationControlInterface
{
public:
	void deformationControlInit(void) override;
	void deformationControlIdle(void) override;
	void deformationControlStop(void) override;
	
	void deformationControlGraspingT1S3(void) override;
	void deformationControlPerformT1S3(void) override;
};

#endif//SPECIFIC_DEFORMATION_CONTROL_HPP
