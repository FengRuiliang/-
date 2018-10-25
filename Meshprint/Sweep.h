#pragma once
#include "Segment.h"
class Event
{
public:
	Event() {};
	~Event() {};
	Event(Vec2f p) { pos = p; };
	Event(Vec3f p) { pos =Vec2f( p.x(),p.y()); };
	std::vector<Segment*> U;//all segments whose upper endpoint is this point
	std::vector<Segment*> L;//all segments whose lower endpoint is this point
 	std::vector<Segment*> C;// all segments who contain this point
	Vec2f pos;
};


class Sweep
{
private:
	std::vector<Segment*> segments;// the set which contains all segment
	bool stronglyintersec = false;
	
	
	struct ComSeg
	{
		bool	operator ()(Segment* left, Segment* right)
		{
			if (abs(left->get_pc().x() - right->get_pc().x()) < 1e-3)
			{
				return left->get_v2().x() < right->get_v2().x();
			}

			return left->get_pc().x() < right->get_pc().x();
		}

	};
	std::set<Segment*,ComSeg > dbtree;
	struct ComEvent 
	{
		bool operator()(Event* left, Event* right)
		
		{
			if (left->pos.y() > right->pos.y())
			{
				return true;
			}
			else if (abs(left->pos.y() - right->pos.y()) < 1e-3)
			{
				return left->pos.x() < right->pos.x();
			}
			else
			{
				return false;
			}
		}
	};
	std::set<Event *,ComEvent>   events;
	std::pair<Vec2f, std::vector<Segment*>>  handle_event_point(Event* param1);
	void find_new_event(Segment* sl, Segment* sr, Event* param1);
public:

	Sweep();
	~Sweep();
	bool set_strongly_intersection(bool param1) { stronglyintersec = param1; }
	void insert_segment(Segment *seg);
	void find_intersection();
};

