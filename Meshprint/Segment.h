#pragma once

#include "HE_mesh/Vec.h"
#include "Event.h"
using namespace trimesh;
class Segment
{
public:
	Segment() ;
	~Segment() ;
	Segment(vec3 p1, vec3 p2);
	bool operator<(const Segment & right)const   //ÖØÔØ<ÔËËã·û
	{
		return this->_middle.x() < right._middle.x();
		
	}
private:
	vec3 _front,_back,_middle;
	float _angle_belong;
	Segment* _next;
	Segment* _prev;
};



