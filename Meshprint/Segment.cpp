#include "Segment.h"



Segment::Segment()
{
}


Segment::~Segment()
{
	_next = NULL;
	_prev = NULL;
}

Segment::Segment(vec3 p1, vec3 p2)
{
	_front = p1;
	_back = p2;
	_middle = p1.y() > p2.y() ? p1 : p2;
}


