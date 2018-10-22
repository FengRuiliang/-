#pragma once
#include "queue"
#include "Segment.h"
#include "Event.h"
#include <set>
#include "HE_mesh/Vec.h"
class SweepLine
{
private:
	
public:
	SweepLine();
	~SweepLine();
	void insert_segment(Segment segment);

private:
	std::queue<Event> Q;
	std::set<Segment> sweep_line_intersect_segment;
	std::vector<Event*> all_event;
	std::vector<Segment*> all_segment;
};

