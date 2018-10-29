#pragma once
#include <vector>
#include "Segment.h"
#include "Sweep.h"
class Supportor
{
public:
	Supportor();
	~Supportor();
	void add_support_point(std::vector<std::vector<std::vector<Segment*>>> cnts);
private:
	void sample_support_point(std::vector<std::vector<Segment*>> uper, std::vector<std::vector<Segment*>> under);
	float get_sup_length(float angle);
};

