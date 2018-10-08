#pragma once
#include "HE_mesh/Vec.h"

using namespace trimesh;

class CSegment
{
public:
	CSegment() {};
	~CSegment() {};
private:
	vec3 sta_, end_;
	CSegment* next_;
	CSegment* prev_;
};

