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
private:
	void sample_support_point(Paths safe_region, Paths contours, float param3);
};

