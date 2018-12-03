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
	void add_support_point(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	std::vector<Vec3f>* get_sup_points() { return sup_points; }
	std::map<int, std::vector<std::pair<ivec2, ivec2>>>* hatchs;
private:
	std::vector<Vec3f>* sup_points;
};

