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
	std::vector<std::vector<Segment>>* hatchs;
	std::vector<Vec3f>* sup_points;
	std::vector<std::vector<std::vector<Vec3f>>>* polylines;
	std::vector<std::vector<std::vector<Vec3f>>>* minkowskisums;
	void add_supportting_point_for_polyline(std::vector<Vec3f> poly);
	void merge();
	inline void findpolyline(Path target_paths, Paths mink_sum, int num);
	void add_supportting_point_for_hatchs(std::vector<Segment> hatchs);
};

