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
	std::vector<std::vector<Vec3f>>* get_polylines() { return polylines; };
	std::vector<std::vector<std::vector<Vec3f>>>* get_minkowssum() { return minkowskisums; };
	std::map<int, std::vector<std::pair<ivec2, ivec2>>>* hatchs;
private:
	std::vector<Vec3f>* sup_points;
	std::vector<std::vector<Vec3f>>* polylines;
	std::vector<std::vector<std::vector<Vec3f>>>* minkowskisums;
	void add_supportting_point_for_polylines(std::vector<Vec3f> poly);
	void merge();
	void findpolyline(Paths sup_paths, Paths mink_sum);
};

