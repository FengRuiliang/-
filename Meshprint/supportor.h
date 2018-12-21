#pragma once
#include <vector>
#include "Segment.h"
#include "Sweep.h"
#include "slicer.h"
#include "clipper.hpp"
#include "Hatch.h"
using namespace ClipperLib;

class Supportor
{
public:
	Supportor();
	~Supportor();
	void add_supportting_point_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	std::vector<Vec3f>* get_sup_points() { return sup_points; }
	std::vector<std::vector<std::vector<Vec3f>>>* get_polylines() { return polylines; };
	std::vector<std::vector<std::vector<Vec3f>>>* get_minkowssum() { return minkowskisums; };
	std::vector<std::vector<Segment>>* get_hatchs() { return hatchs; }
	
private:
	float PBL = 4.0;// printable bridge length
	float ERR = 0.2;
	int   PBG = 247;//printable bridge gap
	int   OSD = 384;//offset distance
	std::vector<std::vector<Segment>>* hatchs;
	std::vector<Vec3f>* sup_points;
	std::vector<std::vector<std::vector<Vec3f>>>* polylines;
	std::vector<std::vector<std::vector<Vec3f>>>* minkowskisums;
	void add_supportting_point_for_polyline(std::vector<Vec3f> poly);
	void merge();
	inline void findpolyline(Path target_paths, Paths mink_sum, int num);
	void add_supportting_point_for_hatchs(std::vector<Segment> hatchs);
};

