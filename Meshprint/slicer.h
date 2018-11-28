#pragma once
#include "HE_mesh/Vec.h"
#include "HE_mesh/Mesh3D.h"
#include "Segment.h"
class Slicer
{
private:
	Mesh3D* obj;
	float thickness;
	int num;
	std::vector<std::vector<std::vector<Segment*>>>* contours;
	std::vector<bool> slice_need_sup;
public:
	
	Slicer();
	~Slicer();
	Slicer(Mesh3D* tar,float in_thickness=0.09);
	void doslice();
	std::vector<std::vector<std::vector<Segment*>>>* get_contours() { return contours; }
	std::vector<bool>& get_slice_need_sup() { return slice_need_sup; }
};

