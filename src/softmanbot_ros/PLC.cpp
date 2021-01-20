#include "PLC.hpp"
#include <Python.h>

#include <iostream>

static PyObject *pLogixName;
static PyObject *pLogixModule;


//Integration of functions needed to talk with PLC

//The header and IP adress are in the python file plcComm.py

void PLC_Init(void)
{
	Py_Initialize();
	pLogixName = PyUnicode_FromString("plcComm");	
	pLogixModule = PyImport_Import(pLogixName); //For this to work, i added plcComm into pythonpath in file ~/.bashrc
	PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "init");	
	PyObject *pValue = PyObject_CallObject(pFunc, NULL);//NULL instead of py_args if no arguments*/
}

void PLC_End(void)
{
	PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "end");	
	PyObject *pValue = PyObject_CallObject(pFunc, NULL);//NULL instead of py_args if no arguments*/	
	Py_Finalize();
}

void PLC_WriteTag(const std::string &tagName, int valueToWrite)
{
	PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "writeTag");	
	PyObject *py_args = PyTuple_New(2);
	PyTuple_SetItem(py_args, 0, PyUnicode_FromString(tagName.c_str()));				
	PyTuple_SetItem(py_args, 1, PyLong_FromLong(valueToWrite));	
	PyObject *pValue = PyObject_CallObject(pFunc, py_args);//NULL instead of py_args if no arguments*/
}

int PLC_ReadTag(const std::string &tagName)
{	
	PyObject *pFunc = PyObject_GetAttrString(pLogixModule, "readTag");
	PyObject *py_args = PyTuple_New(1);
	PyTuple_SetItem(py_args, 0, PyUnicode_FromString(tagName.c_str()));				
	PyObject *pValue = PyObject_CallObject(pFunc, py_args);//NULL instead of py_args if no arguments*/
	PyObject *readValue = PyObject_GetAttrString(pValue, "Value");		
	int retval = PyLong_AsLong(readValue);
	
	return retval;
}
