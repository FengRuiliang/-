#include "supportor.h"
#include "slicer.h"
#include <vector>
#include "clipper.hpp"
using namespace ClipperLib;
Supportor::Supportor()
{
	sup_points = new std::vector<Vec3f>;
}


Supportor::~Supportor()
{
}

void Supportor::add_support_point(std::vector<std::vector<std::vector<Segment*>>>* cnts, std::vector<bool>& need_sup_sl)
{
	ClipperLib::Clipper cli;
	
	for (int i = 2; i < cnts->size();i++)
	{
		Paths under, upper;
		for (int j = 0; j < cnts->at(i-1).size(); j++)
		{
			Path path;
			for (int k = 0; k < cnts->at(i-1)[j].size(); k++)
			{
				path << IntPoint(cnts->at(i-1)[j][k]->get_v1().x()*1e3,
					cnts->at(i-1)[j][k]->get_v1().y()*1e3);
			}
			under << path;
		}
		for (int j = 0; j < cnts->at(i ).size(); j++)
		{
			Path path;
			for (int k = 0; k < cnts->at(i)[j].size(); k++)
			{
				path << IntPoint(cnts->at(i)[j][k]->get_v1().x()*1e3,
					cnts->at(i)[j][k]->get_v1().y()*1e3);
			}
			upper << path;
		}
		set_angle_of_edge_in_contuor(under, cnts->at(i));
		ClipperOffset off;
		for (int j=0;j<2;j++)
		{
			off.Clear();
			off.AddPaths(upper, jtMiter, etClosedPolygon);
			off.Execute(upper, -j * 510);

		}
		
	}
	

	for (int i = 0; i < need_sup_sl.size(); i++)
	{ 
		sample_support_point(cnts->at(i));
	}
}

float Supportor::get_sup_length(float angle)
{
	if (angle < 10)
	{
		return 1.0 / 3.0;
	}
	else if (angle < 15)
	{
		return 1.0 / 8.0;
	}
	else if (angle < 20)
	{
		return 1.0 / 10.0;
	}
	else if (angle < 25)
	{
		return 1 / 15;
	}
	else if (angle < 30)
	{
		return 1 / 20;
	}
	else
	{
		return 0.001;
	}
}
void Supportor::sample_support_point(std::vector<std::vector<Segment*>> upper)
{
	for each (std::vector<Segment*> poly in upper)
	{

		float sum = 0;
		if (poly.front()->get_angle()<30)
		{
			sum = 1;
		}
		for each(Segment* seg in poly)
		{
			float remain_length = (seg->get_v2() - seg->get_v1()).length();
			float ratio = get_sup_length(seg->get_angle());
			Vec3f dir = seg->get_v2() - seg->get_v1();
			dir.normalize();
			Vec3f p = seg->get_v1();
			while (remain_length>(1-sum)/ratio)
			{
				p += (1 - sum) / ratio*dir;
				sup_points->push_back(p);
				remain_length -= (1 - sum) / ratio;
				sum = 0;
			}
			sum += ratio*remain_length;
		}
		
	}
}

void Supportor::set_angle_of_edge_in_contuor(Paths under, std::vector<std::vector<Segment*>> sl)
{
	ClipperOffset off;
	Paths solution;
	// <30 degree
	off.Clear();
	off.AddPaths(under, jtMiter, etClosedPolygon);
	off.Execute(solution, 156);
	for (int j = 0; j < sl.size(); j++)
	{
		Path path;
		for (int k = 0; k <sl[j].size(); k++)
		{
			IntPoint p1(sl[j][k]->get_v1().x()*1e3,
				sl[j][k]->get_v1().y()*1e3);
			IntPoint p2(sl[j][k]->get_v2().x()*1e3,
				sl[j][k]->get_v2().y()*1e3);
			path << p1;
			bool is_in_1 = false;
			bool is_in_2 = false;
			for (int m = 0; m < solution.size(); m++)
			{
				if (PointInPolygon(p1, solution[m]) == 1)
				{
					is_in_1 = !is_in_1;
				}
				if (PointInPolygon(p2, solution[m]) == 1)
				{
					is_in_2 = !is_in_2;
				}
			}
			if ((!is_in_1) || (!is_in_2))
			{

				sl[j][k]->set_angle(25);
			}
		}
	}
	//<25 degree
	off.Clear();
	off.AddPaths(under, jtMiter, etClosedPolygon);
	off.Execute(solution, 193);
	for (int j = 0; j <sl.size(); j++)
	{
		Path path;
		for (int k = 0; k <sl[j].size(); k++)
		{
			IntPoint p1(sl[j][k]->get_v1().x()*1e3,
				sl[j][k]->get_v1().y()*1e3);
			IntPoint p2(sl[j][k]->get_v2().x()*1e3,
				sl[j][k]->get_v2().y()*1e3);
			path << p1;
			bool is_in_1 = false;
			bool is_in_2 = false;
			for (int m = 0; m < solution.size(); m++)
			{
				if (PointInPolygon(p1, solution[m]) == 1)
				{
					is_in_1 = !is_in_1;
				}
				if (PointInPolygon(p2, solution[m]) == 1)
				{
					is_in_2 = !is_in_2;
				}
			}
			if ((!is_in_1) || (!is_in_2))
			{

				sl[j][k]->set_angle(20);
			}
		}
	}
	//<20 degree
	off.Clear();
	off.AddPaths(under, jtMiter, etClosedPolygon);
	off.Execute(solution, 247);
	for (int j = 0; j <sl.size(); j++)
	{
		Path path;
		for (int k = 0; k <sl[j].size(); k++)
		{
			IntPoint p1(sl[j][k]->get_v1().x()*1e3,
				sl[j][k]->get_v1().y()*1e3);
			IntPoint p2(sl[j][k]->get_v2().x()*1e3,
				sl[j][k]->get_v2().y()*1e3);
			path << p1;
			bool is_in_1 = false;
			bool is_in_2 = false;
			for (int m = 0; m < solution.size(); m++)
			{
				if (PointInPolygon(p1, solution[m]) == 1)
				{
					is_in_1 = !is_in_1;
				}
				if (PointInPolygon(p2, solution[m]) == 1)
				{
					is_in_2 = !is_in_2;
				}
			}
			if ((!is_in_1) || (!is_in_2))
			{

				sl[j][k]->set_angle(15);
			}
		}
	}
	//<15 degree
	off.Clear();
	off.AddPaths(under, jtMiter, etClosedPolygon);
	off.Execute(solution, 336);
	for (int j = 0; j <sl.size(); j++)
	{
		Path path;
		for (int k = 0; k <sl[j].size(); k++)
		{
			IntPoint p1(sl[j][k]->get_v1().x()*1e3,
				sl[j][k]->get_v1().y()*1e3);
			IntPoint p2(sl[j][k]->get_v2().x()*1e3,
				sl[j][k]->get_v2().y()*1e3);
			path << p1;
			bool is_in_1 = false;
			bool is_in_2 = false;
			for (int m = 0; m < solution.size(); m++)
			{
				if (PointInPolygon(p1, solution[m]) == 1)
				{
					is_in_1 = !is_in_1;
				}
				if (PointInPolygon(p2, solution[m]) == 1)
				{
					is_in_2 = !is_in_2;
				}
			}
			if ((!is_in_1) || (!is_in_2))
			{

				sl[j][k]->set_angle(10);
			}
		}
	}
	//<10 degree
	off.Clear();
	off.AddPaths(under, jtMiter, etClosedPolygon);
	off.Execute(solution, 510);
	for (int j = 0; j <sl.size(); j++)
	{
		Path path;
		for (int k = 0; k <sl[j].size(); k++)
		{
			IntPoint p1(sl[j][k]->get_v1().x()*1e3,
				sl[j][k]->get_v1().y()*1e3);
			IntPoint p2(sl[j][k]->get_v2().x()*1e3,
				sl[j][k]->get_v2().y()*1e3);
			path << p1;
			bool is_in_1 = false;
			bool is_in_2 = false;
			for (int m = 0; m < solution.size(); m++)
			{
				if (PointInPolygon(p1, solution[m]) == 1)
				{
					is_in_1 = !is_in_1;
				}
				if (PointInPolygon(p2, solution[m]) == 1)
				{
					is_in_2 = !is_in_2;
				}
			}
			if ((!is_in_1) || (!is_in_2))
			{

				sl[j][k]->set_angle(0);
			}
		}
	}
}
