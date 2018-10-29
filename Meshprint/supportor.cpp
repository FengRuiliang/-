#include "supportor.h"
#include "slicer.h"
#include <vector>
#include "clipper.hpp"
using namespace ClipperLib;
Supportor::Supportor()
{
}


Supportor::~Supportor()
{
}

void Supportor::add_support_point(std::vector<std::vector<std::vector<Segment*>>> cnts)
{
	Clipper clipper;
	ClipperOffset offsetor;
	Paths safe_polygon;
	Path rec;
	rec << IntPoint(-1000000, -1000000) << IntPoint(-1000000, 1000000) << IntPoint(1000000, 1000000) << IntPoint(-1000000, 1000000);
	safe_polygon << rec;
	Paths cur_polygon;
	for (size_t i = 1; i < cnts.size(); i++)
	{

		Path polyline;
		for (int j = 0; j < cnts[i].size(); j++)
		{
			for (int k = 0; k < cnts[i][j].size(); k++)
			{
				polyline << IntPoint(cnts[i][j][k]->get_v1().x()*1e3, cnts[i][j][k]->get_v1().y()*1e3);
			}
		}
		cur_polygon.push_back(polyline);

		Paths overhang;
		clipper.AddPaths(safe_polygon, ptClip, true);
		clipper.AddPaths(cur_polygon, ptSubject, true);
		clipper.Execute(ctDifference, overhang, pftNonZero, pftNonZero);
		if (!overhang.empty())
		{
			sample_support_point(cnts[i], cnts[i - 1]);
			offsetor.Clear();
			offsetor.AddPaths(cur_polygon, jtMiter, etClosedPolygon);
			offsetor.Execute(safe_polygon, 3.0);
		}
		else
		{
			clipper.Clear();
			clipper.AddPaths(safe_polygon, ptClip, true);
			clipper.AddPaths(cur_polygon, ptSubject, true);
			clipper.Execute(ctIntersection, safe_polygon, pftNonZero, pftNonZero);
			offsetor.Clear();
			offsetor.AddPaths(safe_polygon, jtMiter, etClosedPolygon);
			offsetor.Execute(safe_polygon, 3.0);
		}
	}

}

void Supportor::sample_support_point(std::vector<std::vector<Segment*>> uper, std::vector<std::vector<Segment*>> under)
{

	Paths under_paths;
	for each (std::vector<Segment*> var in under)
	{
		Path path;
		for each (Segment* ptr_seg in var)
		{
			path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
		}
		under_paths << path;
	}

	Paths upper_paths;
	for each (std::vector<Segment*> var in uper)
	{
		Path path;
		for each (Segment* ptr_seg in var)
		{
			path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
		}
		upper_paths << path;
	}
	std::vector<std::vector<int>> is_outside_for_upper;	//1 means inside the under polygon,
												//-1 means cross the under polygon,
												//0 means outside the under polygon
	for each (std::vector<Segment*> var in uper)
	{
		std::vector<int> cur_outside;
		for each (Segment* ptr_seg in var)
		{
			IntPoint p_1(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			IntPoint p_2(ptr_seg->get_v2().x()*1e3, ptr_seg->get_v2().y()*1e3);
			int pip1 = 0, pip2 = 0;
			for each (Path path in under_paths)
			{
				pip1 += (int)(PointInPolygon(p_1, path) == 1);
				pip2 += (int)(PointInPolygon(p_2, path) == 1);
			}
			if (pip1 % 2 + pip2 % 2 == 1)
			{
				cur_outside.push_back(-1);
			}
			else if (pip1 % 2 + pip2 % 2 == 0)
			{
				cur_outside.push_back(0);
			}
			else
			{
				cur_outside.push_back(1);
			}
		}
		is_outside_for_upper.push_back(cur_outside);
	}
	for (int i = 0; i < is_outside_for_upper.size(); i++)
	{


	}

	std::vector<std::vector<int>> is_outside_for_under;	//1 means inside with the upper polygon,
														//-1 means cross with the upper polygon,
														//0 means outside with the upper polygon
	for each (std::vector<Segment*> var in under)
	{
		std::vector<int> cur_outside;
		for each (Segment* ptr_seg in var)
		{
			IntPoint p_1(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			IntPoint p_2(ptr_seg->get_v2().x()*1e3, ptr_seg->get_v2().y()*1e3);
			int pip1 = 0, pip2 = 0;
			for each (Path path in upper_paths)
			{
				pip1 += (int)(PointInPolygon(p_1, path) == 1);
				pip2 += (int)(PointInPolygon(p_2, path) == 1);
			}
			if (pip1 % 2 + pip2 % 2 == 1)
			{
				cur_outside.push_back(-1);
			}
			else if (pip1 % 2 + pip2 % 2 == 0)
			{
				cur_outside.push_back(0);
			}
			else
			{
				cur_outside.push_back(1);
			}
		}
		is_outside_for_under.push_back(cur_outside);
	}
	Sweep sweep_line;
	for (int i = 0; i < is_outside_for_upper.size(); i++)
	{
		for (int j = 0; j < is_outside_for_upper[i].size(); j++)
		{
			if (is_outside_for_upper[i][j] == -1)
			{
				sweep_line.insert_segment(uper[i][j]);
			}
		}
	}
	for (int i = 0; i < is_outside_for_under.size(); i++)
	{
		for (int j = 0; j < is_outside_for_under[i].size(); j++)
		{
			if (is_outside_for_under[i][j] == -1)
			{
				sweep_line.insert_segment(under[i][j]);
			}
		}
	}

	sweep_line.find_intersection();
	//sweep_line.set_strongly_intersection(true);
	//define lines which are need to be support
	typedef std::vector < std::pair<Vec2f, std::vector<Segment*>>> Type_itsps;
	typedef std::pair<Vec2f, std::vector<Segment*>>  Type_itsp;
	Type_itsps itsps = sweep_line.get_intersection_points();
	for each (Type_itsp itsp in itsps)
	{
		if (itsp.second.size() == 2)
		{
			Segment* uper_seg, *under_seg;
			if (itsp.second[0]->get_v1().z() - itsp.second[1]->get_v1().z() < -1e-3)
			{
				uper_seg = itsp.second[1];
				under_seg = itsp.second[0];
			}
			else
			{
				uper_seg = itsp.second[0];
				under_seg = itsp.second[1];
			}
			//judge whether the uper is outer segment or inter segment
			Vec3f dir_of_under = under_seg->get_v2() - under_seg->get_v1();
			Vec3f dir_from_uper_to_under = uper_seg->get_v2() - under_seg->get_v1();
			dir_from_uper_to_under.z() = 0;
			dir_of_under.z() = 0;
			if (dir_of_under.cross(dir_from_uper_to_under).z() > 0)
			{

				uper_seg->set_vin(Vec3f(itsp.first.x(), itsp.first.y(), uper_seg->get_v1().z()));
			}
			else
			{

				uper_seg->set_vout(Vec3f(itsp.first.x(), itsp.first.y(), uper_seg->get_v1().z()));

			}


		}
		else
		{
			qDebug() << "itsp contains more than 2 segment";
		}
	}
}
float Supportor::get_sup_length(float angle)
{
	return 1 / (0.94 + pow(0.32, 0.26*angle));
}
void Supportor::sample_support_point(std::vector<Segment*> upper)
{
	std::vector<Vec3f> sup_points;
	float sum = 0;
	for each (Segment* seg in upper)
	{
		if (seg->get_angle() < 30)
		{
			float ratio = get_sup_length(seg->get_angle());
			float length = (seg->get_v2() - seg->get_v1()).length();
			sum += ratio*length;
			if (sum > 1) 
			{
				Vec3f dir = seg->get_v2() - seg->get_v1();
				dir.normalize();
				float next=sum-1;
				sup_points.push_back(seg->get_v1() + (1 - ratio)*dir);
				sum = next;
			}
		}
		else
		{
			sum = 0;
		}
		
	}
}