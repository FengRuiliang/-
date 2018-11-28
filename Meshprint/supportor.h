#pragma once
#include <vector>
#include "Segment.h"
#include "Sweep.h"
class Supportor
{
public:
	Supportor();
	~Supportor();
	void add_support_point(std::vector<std::vector<std::vector<Segment*>>>* cnts);
	std::vector<Vec3f>* get_sup_points() { return sup_points; }
private:
	std::vector<Vec3f>* sup_points;
	void sample_support_point(std::vector<std::vector<Segment*>> upper);
};

