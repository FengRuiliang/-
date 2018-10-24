#include "Sweep.h"



Sweep::Sweep()
{
}


Sweep::~Sweep()
{
}

void Sweep::add_segment(Segment  *seg)
{
	Event *eu, *el;
	if (seg->get_v1().y() > seg->get_v2().y())
	{
		eu = new Event(seg->get_v1());
		el = new Event(seg->get_v2());
	}
	else if (seg->get_v1().y() == seg->get_v2().y())
	{
		if (seg->get_v1().x() > seg->get_v2().x())
		{
			eu = new Event(seg->get_v1());
			el = new Event(seg->get_v2());
		}
		else
		{
			eu = new Event(seg->get_v2());
			el = new Event(seg->get_v1());
		}
	}
	else
	{
		eu = new Event(seg->get_v2());
		el = new Event(seg->get_v1());
	}
	auto reu = set_Z.insert(eu);
	auto rel = set_Z.insert(el);
	(*reu.first)->U.push_back(seg);
	(*rel.first)->L.push_back(seg);
	if (!reu.second)
	{
		delete eu;
	}
	if (!rel.second)
	{
		delete el;
	}
	vec_S.push_back(seg);
}

void Sweep::find_intersection()
{
	std::set<Segment* > tree;

}

bool Event::operator<(const Event & eve2)
{ 
	if (this->pos.y() < eve2.pos.y())
	{
		true;
	}
	else if ( this->pos.y()==eve2.pos.y())
	{
		return this->pos.x() < eve2.pos.x();
	}
	else
	{
		return false;
	}
}
