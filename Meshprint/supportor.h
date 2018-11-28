#pragma once
#include <vector>
#include "Segment.h"
#include "Sweep.h"
class Supportor
{
public:
	Supportor();
	~Supportor();
	void add_support_point(std::vector<std::vector<std::vector<Segment*>>>* cnts, std::vector<bool>& need_sup_sl);
	std::vector<Vec3f>* get_sup_points() { return sup_points; }
private:
	std::vector<Vec3f>* sup_points;
	
	void sample_support_point(std::vector<std::vector<Segment*>> uper, std::vector<std::vector<Segment*>> under);
	float get_sup_length(float angle);
	void sample_support_point(std::vector<std::vector<Segment*>> upper);
	void find_sup_region();
	void set_angle_of_edge_in_contuor(Paths under, std::vector<std::vector<Segment*>> sl);
};

