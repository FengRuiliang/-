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


void Supportor::add_support_point(std::vector<std::vector<std::vector<Segment*>>>* cnts)
{
	ClipperLib::ClipperOffset offsetor;
	ClipperLib::Clipper cliper;
	for (int i = 2; i < cnts->size(); i++)
	{
		Paths under_paths;
		for each (std::vector<Segment*> var in cnts->at(i-1))
		{
			Path path;
			for each (Segment* ptr_seg in var)
			{
				path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			}
			under_paths << path;
		}
		offsetor.Clear();
		offsetor.AddPaths(under_paths, jtMiter, etClosedPolygon);
		offsetor.Execute(under_paths, 384);
		for (int j = 0; j < cnts->at(i).size(); j++)
		{
			for (int k = 0; k < cnts->at(i)[j].size(); k++)
			{
				IntPoint p_1(cnts->at(i)[j][k]->get_v1().x()*1e3, cnts->at(i)[j][k]->get_v1().y()*1e3);
				IntPoint p_2(cnts->at(i)[j][k]->get_v2().x()*1e3, cnts->at(i)[j][k]->get_v2().y()*1e3);
				int pip1 = 0, pip2 = 0;
				for each (Path path in under_paths)
				{
					pip1 += (int)(PointInPolygon(p_1, path) == 1);
					pip2 += (int)(PointInPolygon(p_2, path) == 1);
				}
				//1 means inside the under polygon,
				//-1 means cross the under polygon,
				//0 means outside the under polygon
				if (pip1 % 2 + pip2 % 2 != 1)
				{
					cnts->at(i)[j][k]->set_angle(0);
				}
			}
		}
		sample_support_point(cnts->at(i));
	}
}


void Supportor::sample_support_point(std::vector<std::vector<Segment*>> upper)
{
	for each (std::vector<Segment*> poly in upper)
	{
		float length_left = 0;
		for each(Segment* seg in poly)
		{
			if (seg->get_angle() < 21)
			{
				length_left += seg->get_length();
				while (length_left>3.5)
				{
					length_left -= 3.5;
					Vec3f p = seg->get_v2() - length_left*seg->get_normal();
					sup_points->push_back(p);
				}
			}
			else
			{
				length_left = 0;
			}
		}
	}
}
