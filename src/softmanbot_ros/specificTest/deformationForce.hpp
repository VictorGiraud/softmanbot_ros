#include <fstream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "deformationPoints.hpp" 
//This file modelise a square elastic object deformation.
//This deformation is based on force representation applied on the solid

class DeformationForce
{
public:
	DeformationForce(){};
    ~DeformationForce(){};
	DeformationForce(float squish):squishValue(squish){};

	friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
		ar &squishValue;
	}
	float getSquish(void){return squishValue;};
	DeformationPoints getEquivalentPoints(void);
private:
	float squishValue;//de combien l'objet a été comprimé/étiré en gardant son volume. Objet de base : carré 10/10
};

DeformationPoints DeformationForce::getEquivalentPoints(void)
{
	//Create a deformablePoints Object
	int x = 10 / squishValue;
	int y = 10 * squishValue;	

	Point tl(0, y);
	Point tr(x, y);
	Point bl(0, 0);
	Point br(x, 0);

	return DeformationPoints(tl, tr, bl, br); 	
}
