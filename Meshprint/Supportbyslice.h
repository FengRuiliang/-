#pragma once
#include "SliceCut.h"
#include "clipper.hpp"
using namespace ClipperLib;

class Supportbyslice
{
public:
	Supportbyslice();
	~Supportbyslice();
	void construct_by_slices(SliceCut* target);
	void sample_support_point(std::vector < std::vector<cutLine>* > safe_region, std::vector<std::vector<cutLine>*> contours);
private:
};

