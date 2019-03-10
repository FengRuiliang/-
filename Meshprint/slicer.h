#pragma once
#include "HE_mesh/Vec.h"
#include "HE_mesh/Mesh3D.h"
#include "Segment.h"
#include "clipper.hpp"
using namespace ClipperLib;
class Slicer
{
private:
	Mesh3D* obj;
	float thickness;
	int num;
	std::vector<std::vector<std::vector<Segment*>>>* contours;
	std::vector<bool> slice_need_sup;
	std::vector<Paths>* c_paths;
public:
	
	Slicer();
	~Slicer();
	Slicer(Mesh3D* tar);
	void doslice();
	std::vector<std::vector<std::vector<Segment*>>>* get_contours() { return contours; }
	std::vector<Paths>* get_c_paths() { return c_paths; }
	std::vector<bool>& get_slice_need_sup() { return slice_need_sup; }
	void offset();
};

