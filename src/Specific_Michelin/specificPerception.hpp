#ifndef SPECIFIC_PERCEPTION_HPP
#define SPECIFIC_PERCEPTION_HPP

#include "genericPerception.hpp"

perceptionInterface& 	getPerceptionInterface		(void);
void					specificPerceptionRosInit	(void);


//Custom declaration of inherited interface


class michelinPerceptionInterface: public perceptionInterface
{
public:
	void perceptionInit(void) override;
	void perceptionIdle(void) override;
	void perceptionStop(void) override;
	
	void perceptionGraspingT1S3(void) override;
	void perceptionPerformT1S3(void) override;
};


#endif //SPECIFIC_PERCEPTION_HPP
