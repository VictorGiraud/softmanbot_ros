#pragma once

#include <fstream>

// include headers that implement a archive in simple text format
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>

//This file modelise a square elastic object deformation.
//This deformation is based on the 4 corners position

typedef enum
{
	POINTS,
	FORCE
}deformationType;

class Point
{
public:
	Point(){};
	~Point(){};
	Point(float abs, float ord):x(abs), y(ord){};

	friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & x;
        ar & y;
	}
	void move(float x, float y);
	float getCoordinateX(void){return x;};
	float getCoordinateY(void){return y;};
private:
	float x;
	float y;
};

class DeformationPoints
{
public:
	DeformationPoints(){};
    ~DeformationPoints(){};
	DeformationPoints(Point tl, Point tr, Point bl, Point br):topLeft(tl), topRight(tr), botLeft(bl), botRight(br){};


	friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & topLeft;
        ar & topRight;
		ar & botLeft;
		ar & botRight;
	}

	Point getPointTL(void){return topLeft;};
	Point getPointTR(void){return topRight;};
	Point getPointBL(void){return botLeft;};
	Point getPointBR(void){return botRight;};
	void moveTL(float x, float y);
	void moveTR(float x, float y);
	void moveBL(float x, float y);
	void moveBR(float x, float y);
		
private:
	Point topLeft;
	Point topRight;
	Point botLeft;
	Point botRight;
};


void Point::move(float xdep, float ydep)
{
	x += xdep;
	y += ydep;
}

void DeformationPoints::moveTL(float x, float y)
{
	topLeft.move(x,y);
}

void DeformationPoints::moveTR(float x, float y)
{
	topRight.move(x,y);
}

void DeformationPoints::moveBL(float x, float y)
{
	botLeft.move(x,y);
}

void DeformationPoints::moveBR(float x, float y)
{
	botRight.move(x,y);
}

