#include "AMObject.h"



AMObject::AMObject()
{
	ptr_mesh_ = NULL;
	mycut = NULL;
	mycutsup = NULL;
	myhatch = NULL;
	myhatchsup = NULL;
	ptr_support_ = NULL;
}


AMObject::~AMObject()
{
	delete ptr_mesh_;
	delete ptr_support_;
	delete mycutsup;
	delete mycut;
	delete myhatchsup;
	delete myhatch;
}
