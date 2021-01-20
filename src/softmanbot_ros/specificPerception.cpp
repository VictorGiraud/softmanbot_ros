#include "specificPerception.hpp"
#include <Python.h>

#include "zhelpers.hpp"

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"

#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <iostream>
#include <sstream>

#include "genericLogic.hpp"

#include "iostream"
#include "sstream"

void testPython(void);
void testZmq(void);

PyObject *pLogixName;
PyObject *pLogixModule;

static ros::Publisher specificSensorPub;


/*void specificPerception_init(void)
{
	//Initialising the publisher object
    ros::NodeHandle nh;
	specificSensorPub = nh.advertise<std_msgs::String>("specific_perception", 1000);

	testZmq();//testing for communication with Nicolas
	//testPython();
}*/

perceptionInterface& getPerceptionInterface(void)
{
	static michelinPerceptionInterface retval;
	return retval;
}

void michelinPerceptionInterface::perceptionInit(void)
{
	std::cout << "Call polymorphed version of the interface : Init" << std::endl;
}

void michelinPerceptionInterface::perceptionIdle(void)
{
	std::cout << "Call polymorphed version of the interface : Idle" << std::endl;
}

void michelinPerceptionInterface::perceptionStop(void)
{
	std::cout << "Call polymorphed version of the interface : Stop" << std::endl;
}

void michelinPerceptionInterface::perceptionGraspingT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : GraspingT1S3" << std::endl;
}

void michelinPerceptionInterface::perceptionPerformT1S3(void)
{
	std::cout << "Call polymorphed version of the interface : PerformT1S3" << std::endl;
}	


void testPython(void)//TODO : create an object 
{

	Py_Initialize();
	
	pLogixName = PyUnicode_FromString("plcComm");	
	pLogixModule = PyImport_Import(pLogixName); //For this to work, i added plcComm into pythonpath in file ~/.bashrc
	
	if(pLogixModule != NULL)
	{
		std::cout << "moduleImported" << std::endl;
		/*PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "writeTag");
		PyObject *py_args = PyTuple_New(2);
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("ZZ_VIC"));				
		PyTuple_SetItem(py_args, 1, PyLong_FromLong(7));	
		PyObject *pValue = PyObject_CallObject(pFunc, py_args);//NULL instead of py_args if no arguments*/
		
		PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "readTag");
		PyObject *py_args = PyTuple_New(1);
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("ZZ_VIC"));				
		PyObject *pValue = PyObject_CallObject(pFunc, py_args);//NULL instead of py_args if no arguments*/
		PyObject *readValue = PyObject_GetAttrString(pValue, "Value");		
	
		std::cout << "Read value " << PyLong_AsLong(readValue) << std::endl;
		//PyObject *py_args = PyTuple_New(2);				
		//PyTuple_SetItem(py_args, 0, PyLong_FromLong(6));//PyUnicode_FromString; PyFloat_AsDouble...
		//PyTuple_SetItem(py_args, 1, PyLong_FromLong(7));	
		//std::cout << "6 x 7 :" << PyLong_AsLong(pValue) << std::endl;
	}
	else
	{
		std::cout << "pModule nullptr" << std::endl;
		//TODO : send an error of fail initialization	
	}
	Py_Finalize();
	return;
	/*PyRun_SimpleString("import sys");
	PyRun_SimpleString("print(\"Python talking to snakes.\")");*/
	
	/*PyRun_SimpleString("import sys");	
	PyRun_SimpleString("from pylogix import PLC");
	PyRun_SimpleString("with PLC() as comm:");
	PyRun_SimpleString("comm.IPAddress = '192.168.1.100'");
    PyRun_SimpleString("ret = comm.Read('Program:PC_CommonPart.R010_ConsVitesse')");//R078_URD.UR_Order_done
    PyRun_SimpleString("print(ret.TagName, ret.Value, ret.Status)");*/

	/*if(pModule != NULL)
	{
		PyObject *pFunc = PyObject_GetAttrString(pModule, "getInteger");
		PyObject *py_args = PyTuple_New(2);
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("6"));
		PyTuple_SetItem(py_args, 0, PyUnicode_FromString("7"));	
		PyObject *pValue = PyObject_CallObject(pFunc, py_args);
		std::cout << "6 x 7 :" << PyLong_AsLong(pValue) << std::endl;
	}
	*/

	/*
		PyObject *pFunc = PyObject_GetAttrString(pModule, "getInteger");
		std::cout << "AttrString gotten" << std::endl;
		PyObject *pValue = PyObject_CallObject(pFunc, NULL);
		std::cout << "return value :" << PyLong_AsLong(pValue) << std::endl;
	*/
}

void testZmq(void)
{
	zmq::context_t context(1);
    zmq::socket_t subscriber (context, ZMQ_SUB);

    std::cout << "trying to connect :" << std::endl;

    subscriber.connect("tcp://192.168.1.120:5563");
    std::cout << "Connection OK :" << std::endl;

    subscriber.setsockopt( ZMQ_SUBSCRIBE, "", 0);

	while(1)
	{
		//  Read message contents
        std::string contents = s_recv (subscriber);		

		std::stringstream ss(contents);
		/*pose tarpos;
		ss >> tarpos.x;
		ss >> tarpos.y;
		ss >> tarpos.z;
		ss >> tarpos.roll;
		ss >> tarpos.pitch;
		ss >> tarpos.yaw;

		std::cout << "received this :" << tarpos.x << " " << tarpos.y << " " << tarpos.z << " " << tarpos.roll << " "<< tarpos.pitch << " " << tarpos.yaw << " " << std::endl; 
	  	*/		
		float errorReceived;
		ss >> errorReceived;

		std::stringstream ssend;
		boost::archive::text_oarchive oa(ssend);
		oa << errorReceived;
		
		std::cout << "received sthg :" << errorReceived << std::endl;

		std_msgs::String msg;
		msg.data = ssend.str();
		specificSensorPub.publish(msg);
	}


    return;
}

void specificPerceptionRosInit(void)
{
	return;
}


/*
    while (1) {

        //  Read envelope with address
        //std::string address = s_recv (subscriber);
        //  Read message contents
        std::string contents = s_recv (subscriber);
        
		std::stringstream ss(contents);
		
		// Running loop till the end of the stream
		std::string temp; 
		float found;
		float sum = 0; 
		while (!ss.eof()) { 
	  
		    // extracting word by word from stream
		    ss >> temp; 
	  
		    // Checking the given word is integer or not
		    if (std::stringstream(temp) >> found) 
		        std::cout << "found this :" << found << " "; 
	  			sum += found;
		    // To save from space at the end of string 
		    temp = ""; 
		} 

        std::cout << "original contents :" << contents << " sum  : " << sum << std::endl;
    }

*/

/*
from pylogix import PLC
with PLC() as comm:
    comm.IPAddress = '192.168.1.100'
    ret = comm.Read('Program:PC_CommonPart.R010_ConsVitesse')
    print(ret.TagName, ret.Value, ret.Status)*/
