#pragma once
#include "Segment.h"
class Event
{
public:
	Event() {};
	~Event() {};
	Event(Vec2f p) { pos = p; };
	Event(Vec3f p) { pos =Vec2f( p.x(),p.y()); };
	bool operator < ( const Event &eve2);
	std::vector<Segment*> U;//all segments whose upper endpoint is this point
	std::vector<Segment*> L;//all segments whose lower endpoint is this point
 	std::vector<Segment*> C;// all segments who contain this point
	Vec2f pos;
};


class Sweep
{
private:
	std::vector<Segment*> vec_S;// the set of all segment
	std::set<Event *>   set_Z;
public:
	Sweep();
	~Sweep();
	void add_segment(Segment *seg);
	void find_intersection();
};

