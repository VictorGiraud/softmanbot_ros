#pragma once

#include <fstream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

#include "deformationPoints.hpp" 
//This file modelise a square elastic object deformation.
//This deformation is based on force representation applied on the solid


class MSD
{
public:
	MSD():mass(1.0), spring(3.0), springRestLength(15.0), damper(1.0) {};
	~MSD(){};
	friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
		ar &mass;
		ar &spring;
		ar &damper;
	}
	float getMass(void){return mass;};
	float getSpring(void){return spring;};
	float getSpringRestLength(void){return springRestLength;};
	float getDamper(void){return damper;};

	float setSpringRestLength(float val){springRestLength = val;};
	float setSpring(float val){spring = val;};
private:
	float mass;
	float spring;
	float springRestLength;
	float damper;	
};

class DeformationMSD
{
public:
	DeformationMSD(){
		diag1MSD.setSpringRestLength(10.60);
		diag2MSD.setSpringRestLength(10.60);	
	};
    ~DeformationMSD(){};
	
	friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
		ar &dp;
		ar &topMSD;
		ar &botMSD;
		ar &leftMSD;
		ar &rightMSD;
	}
	
	DeformationPoints dp;
	MSD topMSD;
	MSD botMSD;
	MSD leftMSD;
	MSD rightMSD;
	MSD diag1MSD;
	MSD diag2MSD;	
private:
	
};

class targetPosition
{
public:
	targetPosition(){};
	targetPosition(int x, int y):posx(x),posy(y){};
	~targetPosition(){};
	int posx;
	int posy; 		
friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
		ar &posx;
		ar &posy;
	}
private:
};
