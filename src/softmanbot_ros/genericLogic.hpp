#pragma once

#include <fstream>

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>


typedef enum
{
	IDLE,
	INIT,
	STOP,

	GRASPINGT1S1,
	PERFORMT1S1,
	GRASPINGT1S2,
	PERFORMT1S2,
	GRASPINGT1S3,
	PERFORMT1S3,

	GRASPINGT2S1,
	PERFORMT2S1,
	GRASPINGT2S2,
	PERFORMT2S2,
	GRASPINGT2S3,
	PERFORMT2S3,
	GRASPINGT2S4,
	PERFORMT2S4,

	GRASPINGT3S1,
	PERFORMT3S1,
	GRASPINGT3S2,
	PERFORMT3S2,
	GRASPINGT3S3,
	PERFORMT3S3,	
}softmanbotState;

typedef enum
{
	SKILLED_WORKCELL,
	DEFORMATION_CONTROL,
	PERCEPTION,
	SUPERVISOR
}nodeID;

class pose
{
public:
	float x;
	float y;
	float z;
	float roll;
	float pitch;
	float yaw;
	pose(){};
	~pose(){};
	friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
		ar & z;
		ar & roll;
        ar & pitch;
		ar & yaw;
	}
};
