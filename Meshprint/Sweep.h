#pragma once
#include "HE_mesh/Vec.h"
#include <queue>
#include "SliceCut.h"
class IntersectionPolygon
{

public:
	IntersectionPolygon();
	~IntersectionPolygon();
private: 
	std::queue<trimesh::vec3> events;
	std::vector<cutLine*>      segments;
};

