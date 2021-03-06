#pragma once

#include "genericLogic.hpp"
#include "genericSkilledWorkcell.hpp"

#include <vector>
#include <string>

skilledWorkcellInterface& 	getSkilledWorkcellInterface		(void);
nodeID 						specificSkilledWorkcellGetMaster(void);
void						specificSkilledWorkcellRosInit	(void);

void						specificSkilledWorkcell_setSensor(std::string);

void						specificSkilledWorkcellControl(std::string commandString, std::string sensorString);

//Custom declaration of inherited interface
class michelinSkilledWorkcellInterface: public skilledWorkcellInterface
{
public:
	void skilledWorkcellInit(void) override;
	void skilledWorkcellIdle(void) override;
	void skilledWorkcellStop(void) override;
	
	void skilledWorkcellGraspingT1S3(void) override; //Precise joining of 2 layers
	void skilledWorkcellPerformT1S3(void) override;
};

