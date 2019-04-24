#include "Segment.h"

Vec3f Segment::get_normal()
{ 
	Vec3f nor = v2 - v1;
	nor.normalize();
	return nor;
}

void Segment::set_cut_edge(HE_edge * ecur)
{
	edge_ = ecur;
}
