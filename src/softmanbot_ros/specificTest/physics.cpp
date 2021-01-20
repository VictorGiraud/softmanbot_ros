//This file will act as a placeholder to demonstrate all different kind of specific signals you need to send on your softmanbot application.
//

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <sstream>

#include "deformationPoints.hpp"
#include "deformationForce.hpp"
#include "deformationMSD.hpp"

#include "genericLogic.hpp"

//Using SDL and standard IO
#include <SDL2/SDL.h>
#include <stdio.h>

#include <cmath>
#include <vector>
#include <boost/serialization/vector.hpp>

typedef enum
{
	NO_GRASP,
	TLGRASP,
	TRGRASP,
	BLGRASP,
	BRGRASP
}grasp_possibilities;

//Screen dimension constants
const int SCREEN_WIDTH = 640;
const int SCREEN_HEIGHT = 480;

SDL_Window* window = NULL;
SDL_Surface* screenSurface = NULL;
    
static DeformationMSD model;
static targetPosition tarpos;

int leftMousex = 10, leftMousey = 10;
int rightMousex = 630, rightMousey = 10;

static grasp_possibilities firstArmGrasp;
static grasp_possibilities secondArmGrasp;

static ros::Publisher specificSensorPub;

static SDL_Rect pointsToRect(Point pointToTransform);
static void model_update(void);
static void model_sensor_simulate(void);
static void robotPositionCallback(const std_msgs::String::ConstPtr& msg);

void Line( float x1, float y1, float x2, float y2);
void SetPixel(int x, int y);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");
    ros::NodeHandle nh;
  	
	//Initialising the publisher object
	specificSensorPub = nh.advertise<std_msgs::String>("specific_perception", 1000);

	//create a subscriber object
	//ros::Subscriber sub = nh.subscribe("robot_position", 500, robotPositionCallback);
	ros::Subscriber sub = nh.subscribe("currentRobotPose", 500, robotPositionCallback);

	//Initialize SDL
    if( SDL_Init( SDL_INIT_VIDEO ) < 0 )
    {
        printf( "SDL could not initialize! SDL_Error: %s\n", SDL_GetError() );
    }
  	else
    {
        //Create window
        window = SDL_CreateWindow( "SDL Tutorial", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN );
        if( window == NULL )
        {
            printf( "Window could not be created! SDL_Error: %s\n", SDL_GetError() );
        }
		else
        {
			///Get window surface
            screenSurface = SDL_GetWindowSurface( window );
			//Fill the surface white
            SDL_FillRect( screenSurface, NULL, SDL_MapRGB( screenSurface->format, 0xFF, 0xFF, 0xFF ) );
            
            //Update the surface
            SDL_UpdateWindowSurface( window );
        }
	}
	
	
	/*model.dp.moveTL(0, 10);
	model.dp.moveTR(10, 0);
	model.dp.moveBL(-10, 0);
 	model.dp.moveBR(0, -10);*/

	model.dp.moveTL(-5, 10);
	model.dp.moveTR(5, 10);
	model.dp.moveBL(-5, -10);
 	model.dp.moveBR(5, -10);
	
	//Loop at 50Hz until node shut down
	ros::Rate rate(50);	
	bool quit = false;
	firstArmGrasp = NO_GRASP;

	
	while((ros::ok()) && !quit)
	{
		
		SDL_Event e;
		//Handle events on queue
        while( SDL_PollEvent( &e ) != 0 )
        {
            //Clicked on cross
            if( e.type == SDL_QUIT )
            {
                quit = true;
            }
			if(e.type == SDL_MOUSEBUTTONDOWN)
			{
				if(e.button.button == SDL_BUTTON_LEFT)
				{
					firstArmGrasp = TLGRASP;
				 	//Get mouse position
        			SDL_GetMouseState( &leftMousex, &leftMousey );
				}				
				if(e.button.button == SDL_BUTTON_RIGHT)
				{
					secondArmGrasp = TRGRASP;
				 	//Get mouse position
        			SDL_GetMouseState( &rightMousex, &rightMousey );
				
				}
			}
			if( e.type == SDL_KEYDOWN )
			{
				switch( e.key.keysym.sym )
	            {
					case SDLK_s:
						firstArmGrasp = NO_GRASP;
						break;
					case SDLK_d:
						secondArmGrasp = NO_GRASP;
						break;

					default:
						break;	
				}
			}			
        }
		
		//Receive Robot order
		//TODO	
		model_update();

		//Send informations to softmanbot
		//Simulate sensors info
		model_sensor_simulate();

		//Fill the surface white
		SDL_FillRect( screenSurface, NULL, SDL_MapRGB( screenSurface->format, 0xFF, 0xFF, 0xFF ) );

		//Load floor image
		SDL_Surface *floorImage = SDL_LoadBMP( "/home/victor/Bureau/SoftManBot Project/softmanbot_ros/src/softmanbot_ros/floor.bmp" );
		if( floorImage == NULL )
		{
		    printf( "Unable to load image! SDL Error: %s\n", SDL_GetError() );
		}
		SDL_Surface *HookImage = SDL_LoadBMP( "/home/victor/Bureau/SoftManBot Project/softmanbot_ros/src/softmanbot_ros/Hook.bmp" );
		if( HookImage == NULL )
		{
		    printf( "Unable to load image! SDL Error: %s\n", SDL_GetError() );
		}


		 //Apply the image
		SDL_Rect destination;
		destination.x = 0;
		destination.y = 400;

		SDL_BlitSurface( floorImage, NULL, screenSurface, &destination);
	
		Point tl,tr, bl, br;
	    tl = model.dp.getPointTL();
		tr = model.dp.getPointTR();
		bl = model.dp.getPointBL();
		br = model.dp.getPointBR();
	
		SDL_Rect rtl = pointsToRect(tl);
		SDL_Rect rtr = pointsToRect(tr);
		SDL_Rect rbl = pointsToRect(bl);
		SDL_Rect rbr = pointsToRect(br);

		if(TLGRASP == firstArmGrasp)
		{
			//Affichage de la destination du curseur
			SDL_Rect target;
			target.x = leftMousex - 2;
			target.y = leftMousey - 2;

			target.w = 5;
			target.h = 5;
			SDL_FillRect(screenSurface, &target, SDL_MapRGB(screenSurface->format, 0x00, 0xFF, 0x00));


			destination = rtl;
			destination.x -= 13;
			destination.y -= 28;
			
			SDL_BlitSurface( HookImage, NULL, screenSurface, &destination);
		}

		if(TRGRASP == secondArmGrasp)
		{
			//Affichage de la destination du curseur
			SDL_Rect target;
			target.x = rightMousex - 2;
			target.y = rightMousey - 2;

			target.w = 5;
			target.h = 5;
			SDL_FillRect(screenSurface, &target, SDL_MapRGB(screenSurface->format, 0x00, 0xFF, 0xFF));


			destination = rtr;
			destination.x -= 13;
			destination.y -= 28;
			
			SDL_BlitSurface( HookImage, NULL, screenSurface, &destination);
		}


		SDL_FillRect(screenSurface, &rtl, SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0x00));
		SDL_FillRect(screenSurface, &rtr, SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0x00));
		SDL_FillRect(screenSurface, &rbl, SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0x00));
		SDL_FillRect(screenSurface, &rbr, SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0x00));
	
		Line(rtl.x, rtl.y, rtr.x, rtr.y);
		Line(rtr.x, rtr.y, rbr.x, rbr.y);
		Line(rbr.x, rbr.y, rbl.x, rbl.y);
		Line(rbl.x, rbl.y, rtl.x, rtl.y);

		SDL_UpdateWindowSurface( window );

		SDL_FreeSurface(floorImage);
		
		ros::spinOnce();
		rate.sleep();

		//ros::spinOnce();

	} 
	return 0;
}

static SDL_Rect pointsToRect(Point pointToTransform)
{
	SDL_Rect retval;

	int originX = SCREEN_WIDTH/2;
	int originY = SCREEN_HEIGHT/2;

	retval.x = 10 * pointToTransform.getCoordinateX() + originX - 3;
	retval.y = - 10 * pointToTransform.getCoordinateY() + originY - 3;
	retval.w = 5;
	retval.h = 5;

	return retval;
}

static void model_update(void)
{
	Point TL = model.dp.getPointTL();
	Point TR = model.dp.getPointTR();
	Point BL = model.dp.getPointBL();
	Point BR = model.dp.getPointBR();

	float tlxdelta, tlydelta, trxdelta, trydelta, blxdelta, blydelta, brxdelta, brydelta;

	float distP12, distP13, distP42, distP43, distP14, distP23;

	distP12 = sqrt(pow(TL.getCoordinateX() - TR.getCoordinateX(), 2.0) + pow(TL.getCoordinateY() - TR.getCoordinateY(), 2.0));
	distP13 = sqrt(pow(TL.getCoordinateX() - BL.getCoordinateX(), 2.0) + pow(TL.getCoordinateY() - BL.getCoordinateY(), 2.0));
	distP42 = sqrt(pow(BR.getCoordinateX() - TR.getCoordinateX(), 2.0) + pow(BR.getCoordinateY() - TR.getCoordinateY(), 2.0));
	distP43 = sqrt(pow(BR.getCoordinateX() - BL.getCoordinateX(), 2.0) + pow(BR.getCoordinateY() - BL.getCoordinateY(), 2.0));
	distP14 = sqrt(pow(TL.getCoordinateX() - BR.getCoordinateX(), 2.0) + pow(TL.getCoordinateY() - BR.getCoordinateY(), 2.0));
	distP23 = sqrt(pow(TR.getCoordinateX() - BL.getCoordinateX(), 2.0) + pow(TR.getCoordinateY() - BL.getCoordinateY(), 2.0));
	
	float P12cosalpha =  (TL.getCoordinateX() - TR.getCoordinateX())/distP12;
	float P12sinalpha =  (TL.getCoordinateY() - TR.getCoordinateY())/distP12;

	float P13cosalpha =  (TL.getCoordinateY() - BL.getCoordinateY())/distP13;
	float P13sinalpha =  (TL.getCoordinateX() - BL.getCoordinateX())/distP13;

	float P42cosalpha =  (BR.getCoordinateX() - TR.getCoordinateX())/distP42;
	float P42sinalpha =  (BR.getCoordinateY() - TR.getCoordinateY())/distP42;

	float P43cosalpha =  (BR.getCoordinateX() - BL.getCoordinateX())/distP43;
	float P43sinalpha =  (BR.getCoordinateY() - BL.getCoordinateY())/distP43;

	float P14cosalpha =  (TL.getCoordinateX() - BR.getCoordinateX())/distP14;
	float P14sinalpha =  (TL.getCoordinateY() - BR.getCoordinateY())/distP14;

	float P23cosalpha =  (TR.getCoordinateY() - BL.getCoordinateY())/distP23;
	float P23sinalpha =  (TR.getCoordinateX() - BL.getCoordinateX())/distP23;

	float F12 = model.topMSD.getSpring()   *(model.topMSD.getSpringRestLength()   - distP12) * 0.02;//1/50 : time factor
	float F13 = model.leftMSD.getSpring()  *(model.leftMSD.getSpringRestLength()  - distP13) * 0.02;//1/50 : time factor
	float F42 = model.rightMSD.getSpring() *(model.rightMSD.getSpringRestLength() - distP42) * 0.02;//1/50 : time factor
	float F43 = model.botMSD.getSpring()   *(model.botMSD.getSpringRestLength()   - distP43) * 0.02;//1/50 : time factor
	float F14 = model.diag1MSD.getSpring() *(model.diag1MSD.getSpringRestLength() - distP14) * 0.02;//1/50 : time factor
	float F23 = model.diag2MSD.getSpring() *(model.diag2MSD.getSpringRestLength() - distP23) * 0.02;//1/50 : time factor
	

	//just spring physics yet	
	tlxdelta = F12 * P12cosalpha + F13 * P13sinalpha + F14 * P14cosalpha;
	blxdelta = - F43 * P43cosalpha - F13 * P13sinalpha - F23 * P23sinalpha;
	trxdelta = - F12 * P12cosalpha - F42 * P42cosalpha + F23 * P23sinalpha;
	brxdelta = F43 * P43cosalpha + F42 * P42cosalpha - F14 * P14cosalpha; 
	
	//gravity
	float gravity = - 0.1;
	//Floor collision
	if(pointsToRect(TL).y < 395)
	{		
		tlydelta = F13 * P13cosalpha + F12 * P12sinalpha + F14 * P14sinalpha + gravity;
	} 
	else
	{
		tlydelta = F13 * P13cosalpha + F12 * P12sinalpha + F14 * P14sinalpha;	
	}
	if(pointsToRect(TR).y < 395)
	{
		blydelta = - F13 * P13cosalpha - F43 * P43sinalpha - F23 * P23cosalpha + gravity;
	}
	else
	{
		blydelta = - F13 * P13cosalpha - F43 * P43sinalpha - F23 * P23cosalpha;	
	}
	if(pointsToRect(BL).y < 395)
	{
		trydelta = - F42 * P42sinalpha - F12 * P12sinalpha + F23 * P23cosalpha + gravity;  
	}
	else
	{
		trydelta = - F42 * P42sinalpha - F12 * P12sinalpha + F23 * P23cosalpha;  
	}
	if(pointsToRect(BR).y < 395)
	{
		brydelta = F42 * P42sinalpha + F43 * P43sinalpha - F14 * P14sinalpha + gravity;
	}
	else
	{
		brydelta = F42 * P42sinalpha + F43 * P43sinalpha - F14 * P14sinalpha;
	}
	
	if(TLGRASP == firstArmGrasp)
	{
		tlxdelta = 0;
		tlydelta = 0;
	}

	if(TRGRASP == secondArmGrasp)
	{
		trxdelta = 0;
		trydelta = 0;
	}

	//Floor collision
	if((pointsToRect(TL).y + 10*tlydelta) > 397)
	{		
		tlydelta -= 0.1 * (398 - pointsToRect(TL).y);
	} 
	if((pointsToRect(TR).y + 10*trydelta) > 397)
	{
		trydelta -= 0.1 * (398 - pointsToRect(TR).y);
	}
	if((pointsToRect(BL).y + 10*blydelta) > 397)
	{
		blydelta -= 0.1 * (398 - pointsToRect(BL).y);
	}
	if((pointsToRect(BR).y + 10*brydelta) > 397)
	{
		brydelta -= 0.1 * (398 - pointsToRect(BR).y);
	}

	model.dp.moveTL(tlxdelta, tlydelta);
	model.dp.moveTR(trxdelta,trydelta);
	model.dp.moveBL(blxdelta, blydelta);
 	model.dp.moveBR(brxdelta, brydelta);
}

static void model_sensor_simulate(void)
{
	std::stringstream ss;
	boost::archive::text_oarchive oa(ss);

	oa << model;

	tarpos.posx = leftMousex;
    tarpos.posy = leftMousey;
	oa << tarpos;

	tarpos.posx = rightMousex;
    tarpos.posy = rightMousey;
	oa << tarpos;

	std_msgs::String msg;
	msg.data = ss.str();
	specificSensorPub.publish(msg);

}

static void robotPositionCallback(const std_msgs::String::ConstPtr& msg)
{
	std::stringstream ss(msg->data);
	boost::archive::text_iarchive ia(ss);
	
	std::vector<pose> targetPoses;
	ia >> targetPoses;

	pose targetPose = targetPoses[0];

	if(TLGRASP == firstArmGrasp)
	{	
		model.dp.moveTL(targetPose.x, -targetPose.y);
	}
	
	targetPose = targetPoses[1];
	if(TRGRASP == secondArmGrasp)
	{	
		model.dp.moveTR(targetPose.x, -targetPose.y);
	}
}

void Line( float x1, float y1, float x2, float y2)
{
    // Bresenham's line algorithm
    const bool steep = (fabs(y2 - y1) > fabs(x2 - x1));
    if(steep)
    {
        std::swap(x1, y1);
        std::swap(x2, y2);
    }

    if(x1 > x2)
    {
        std::swap(x1, x2);
        std::swap(y1, y2);
    }

    const float dx = x2 - x1;
    const float dy = fabs(y2 - y1);

    float error = dx / 2.0f;
    const int ystep = (y1 < y2) ? 1 : -1;
    int y = (int)y1;

    const int maxX = (int)x2;

    for(int x=(int)x1; x<maxX; x++)
    {
        if(steep)
        {
            SetPixel(y,x);
        }
        else
        {
            SetPixel(x,y);
        }

        error -= dy;
        if(error < 0)
        {
            y += ystep;
            error += dx;
        }
    }
}

void SetPixel(int x, int y)
{
	SDL_Rect cheat;
	cheat.x = x +2;
	cheat.y = y+2;
	cheat.w = 1;
	cheat.h = 1;
	SDL_FillRect(screenSurface, &cheat, SDL_MapRGB(screenSurface->format, 0x00, 0x00, 0x00));
		
}
