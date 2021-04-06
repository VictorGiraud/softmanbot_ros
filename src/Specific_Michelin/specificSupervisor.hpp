#ifndef SPECIFIC_SUPERVISOR_HPP
#define SPECIFIC_SUPERVISOR_HPP

#include <ros/ros.h>

#include "genericLogic.hpp"
#include "genericSupervisor.hpp"

//All specific should get this prototypes needed by generic
softmanbotState 		state_machine(void);
supervisoryInterface& 	getSupervisoryInterface(void);
void					specificSupervisoryRosInit(void);

//Custom declaration of inherited interface
class michelinSupervisoryInterface: public supervisoryInterface
{
public:
	void supervisoryInit(void) override;
	void supervisoryIdle(void) override;
	void supervisoryStop(void) override;
	
	void supervisoryGraspingT1S3(void) override; //Precise joining of 2 layers
	void supervisoryPerformT1S3(void) override;
};


#endif //SPECIFIC_SUPERVISOR_HPP
