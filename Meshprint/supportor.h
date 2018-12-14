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
	std::map<int, std::vector<std::pair<ivec2, ivec2>>>* hatchs;
private:
	std::vector<Vec3f>* sup_points;
	std::vector<std::vector<Vec3f>>* polylines;
	void add_supportting_point_for_polylines(std::vector<Vec3f> poly);
};

