#include "Sweep.h"
#include <queue>




Sweep::Sweep()
{
}


Sweep::~Sweep()
{
	events.clear();
	segments.clear();
}

void Sweep::insert_segment(Segment  *seg)
{
	Event *eu, *el;
	if ((seg->get_v1().y() - seg->get_v2().y())>=1e-3)
	{
		eu = new Event(seg->get_v1());
		el = new Event(seg->get_v2());
		seg->set_pc(seg->get_v1());
		seg->set_vu(seg->get_v1());
		seg->set_vl(seg->get_v2());
		seg->set_pc(seg->get_v1());
	}
	else if (abs(seg->get_v1().y() - seg->get_v2().y())<1e-3)
	{
		seg->set_pc(Vec3f(1e6, seg->get_v1().y(), 0));
		if (seg->get_v1().x() > seg->get_v2().x())
		{
			eu = new Event(seg->get_v1());
			el = new Event(seg->get_v2());
			seg->set_vu(seg->get_v1());
			seg->set_vl(seg->get_v2());
			seg->set_pc(seg->get_v1());
		}
		else
		{
			eu = new Event(seg->get_v2());
			el = new Event(seg->get_v1());
			seg->set_vu(seg->get_v2());
			seg->set_vl(seg->get_v1());
			seg->set_pc(seg->get_v2());
		}
	}
	else
	{
		eu = new Event(seg->get_v2());
		el = new Event(seg->get_v1());
		seg->set_vu(seg->get_v2());
		seg->set_vl(seg->get_v1());
		seg->set_pc(seg->get_v2());
	}
	auto reu = events.insert(eu);
	auto rel = events.insert(el);
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
	segments.push_back(seg);
}

void Sweep::find_intersection()
{
	while (!events.empty())
	{
		Event * e_ =* events.begin();
		events.erase(events.begin());
		intersection_points.push_back(handle_event_point(e_));
		delete e_;
	}
}

std::pair<Vec2f, std::vector<Segment*>>  Sweep::handle_event_point(
	Event* param1//the event is dueling with
)

{
	
	for each (Segment* var in dbtree)
	{
		Vec3f param2;//the intersection point 
		param1->pos.y();
		if (var->get_pc().x()!=1e6)
		{
				param2= (var->get_v1() - var->get_v2())
				*(param1->pos.y() - var->get_v1().y())
				/ (var->get_v1().y() - var->get_v2().y())
				+ var->get_v2();
			var->set_pc(param2);

			// param2 is definitely not equal to vu of var 
			if (abs(param2.x() - param1->pos.x()) <= 1e-3)
			{
				
				param1->C.push_back(var);
			}
		}
		else
		{
			if (var->get_vu().x() > param1->pos.x() && var->get_vl().x() < param1->pos.x())
			{
				param1->C.push_back(var);
			}
		}
		
	}
	
	if (!stronglyintersec&&param1->C.size())
	{
		std::vector<Segment*> intersec;
		intersec.insert(intersec.end(), param1->C.begin(), param1->C.end());
		std::pair<Vec2f, std::vector<Segment*>> pair_intersec(
			Vec2f(param1->pos.x(), param1->pos.y()), intersec);
		return pair_intersec;
	}
	if (stronglyintersec&&param1->U.size()+ param1->L.size()+param1->C.size())
	{
		std::vector<Segment*> intersec;
		intersec.insert(intersec.end(), param1->U.begin(), param1->U.end());
		intersec.insert(intersec.end(), param1->C.begin(), param1->C.end());
		intersec.insert(intersec.end(), param1->L.begin(), param1->L.end());
		std::pair<Vec2f, std::vector<Segment*>> pair_intersec(
			Vec2f(param1->pos.x(), param1->pos.y()), intersec);
		return pair_intersec;
	}
	// remove L and C from dbtree, and add U and C into dbtree
	for each (Segment* var in param1->C)
	{
		var->set_to_erease(true);
	}
	for each (Segment* var in param1->L)
	{
		var->set_to_erease(true);
	}
	for (auto iter=dbtree.begin();iter!=dbtree.end();)
	{
		if ((*iter)->get_to_erease())
		{
			dbtree.erase(iter);
		}
		else
		{
			iter++;
		}
	}
	for (Segment* var:param1->U)
	{
		var->set_to_erease(false);
		dbtree.insert(var);
	}
	for (Segment* var:param1->C)
	{
		var->set_to_erease(false);
		dbtree.insert(var);
	}
	if ((param1->U.size()+param1->C.size())==0)
	{
		Segment* sl,*sr;//when the value of these pointer is null, 
						//it means that there is no segment left or right to the event
		for each (Segment* var in dbtree)
		{
			if (var->get_pc().x() < param1->pos.x() )
			{
				sl = var;
			}
			else
			{
				sr = var;
				break;
			}
		}
		find_new_event(sl, sr, param1);
	}
	else
	{
		Segment* sl, *sr, *sucl, *sucr;//when the value of these pointer is null, 
						 //it means that there is no segment left or right to the event
		for (auto iter=dbtree.begin();iter!=dbtree.end();iter++)
		{
			if ((*iter)->get_pc().x() - param1->pos.x() < -1e-3)
			{
				sl = *iter;
			}
			else
			{
				sucl = *iter;
				find_new_event(sl, sucl, param1);
				for (; (*iter)->get_pc().x() - param1->pos.x() <= 1e-3;)
				{
					sucr = *iter;
					iter++;
				}
				sr = *iter;
				find_new_event(sucr, sr, param1);
			}

		}
	}

}

void Sweep::find_new_event(Segment* sl, Segment* sr, Event* param1)
{
	if (sl!=NULL&&sr!=NULL)
	{
		Vec2f a = sl->get_v1();
		Vec2f b = sl->get_v2();
		Vec2f c = sr->get_v1();
		Vec2f d = sr->get_v2();
		double u, v, w, z;
		u = (c.x() - a.x())*(b.y() - a.y()) - (b.x() - a.x())*(c.y() - a.y());
		v = (d.x() - a.x())*(b.y() - a.y()) - (b.x() - a.x())*(d.y() - a.y());
		w = (a.x() - c.x())*(d.y() - c.y()) - (d.x() - c.x())*(a.y() - c.y());
		z = (b.x() - c.x())*(d.y() - c.y()) - (d.x() - c.x())*(b.y() - c.y());
		if (u*v <= 0.00000001 && w*z <= 0.00000001)
		{
			Vec3f intersec;
			float tmpLeft, tmpRight;
			tmpLeft = (d.x()- c.x()) * (a.y() - b.y()) - (b.x() - a.x()) * (c.y() - d.y());
			tmpRight = (a.y() - c.y()) * (b.x() - a.x()) * (d.x()- c.x()) + c.x()* (d.y() - c.y()) * (b.x() - a.x()) - a.x() * (b.y() - a.y()) * (d.x()- c.x());

			intersec.x() = tmpRight / tmpLeft;

			tmpLeft = (a.x() - b.x()) * (d.y() - c.y()) - (b.y() - a.y()) * (c.x()- d.x());
			tmpRight = b.y() * (a.x() - b.x()) * (d.y() - c.y()) + (d.x()- b.x()) * (d.y() - c.y()) * (a.y() - b.y()) - d.y() * (c.x()- d.x()) * (b.y() - a.y());
			intersec.y() = tmpRight / tmpLeft;
			if (abs(intersec.y() - param1->pos.y()) <= 1e-3)
			{
				if (intersec.x()>param1->pos.x())
				{
					Event * new_eve = new Event(intersec);
					events.insert(new_eve);
				}
			}
			else if (intersec.y()-param1->pos.y()<-1e-3)
			{
				Event * new_eve = new Event(intersec);
				events.insert(new_eve);
			}
		}
	}
}
