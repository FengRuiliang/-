#pragma once
#include "Segment.h"
#include <vector>
#include "HE_mesh/Vec.h"
class Event
{
public:
	Event();
	Event(vec3 pos_);
	~Event();
	vec2 pos() { return _pos; };
	void set_U(Segment* seg) { U_p.push_back(seg); };
private:
	vec2 _pos;
	std::vector<Segment*> U_p;
};

