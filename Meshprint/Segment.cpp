#include "Segment.h"

inline Vec3f Segment::get_normal()
{ 
	Vec3f nor = v2 - v1;
	nor.normalize();
	return nor;
}
