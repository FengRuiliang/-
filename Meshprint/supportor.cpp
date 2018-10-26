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
	rec<< IntPoint(-1000000, -1000000) << IntPoint(-1000000, 1000000) << IntPoint(1000000, 1000000) << IntPoint(-1000000, 1000000);
	safe_polygon << rec;
	Paths cur_polygon;
	for (size_t i = 1; i < cnts.size(); i++)
	{
		
		Path polyline;
		for (int j=0;j<cnts[i].size();j++)
		{
			for (int k=0;k<cnts[i][j].size();k++)
			{
				polyline << IntPoint(cnts[i][j][k]->get_v1().x()*1e3,cnts[i][j][k]->get_v1().y()*1e3);
			}
		}
		cur_polygon.push_back(polyline);

		Paths overhang;
		clipper.AddPaths(safe_polygon, ptClip, true);
		clipper.AddPaths(cur_polygon, ptSubject, true);
		clipper.Execute(ctDifference, overhang, pftNonZero, pftNonZero);
		if (!overhang.empty())
		{
			sample_support_point(cnts[i], cnts[i-1]);
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
	Sweep sweep_line;
	for each (std::vector<Segment*> var in uper)
	{
		for each (Segment*ptr_seg in var)
		{

			sweep_line.insert_segment(ptr_seg);
		}
	}
	for each (std::vector<Segment*> var in under)
	{
		for each (Segment* ptr_seg in var)
		{
			sweep_line.insert_segment(ptr_seg);
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
		if (itsp.second.size()==2)
		{
			Segment* uper_seg, *under_seg;
			if (itsp.second[0]->get_v1().z()-itsp.second[1]->get_v1().z()<-1e-3)
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
				
				uper_seg->set_vin(Vec3f(itsp.first.x(),itsp.first.y(),uper_seg->get_v1().z()));
			}
			else
			{
				
				uper_seg->set_vout(Vec3f(itsp.first.x(),itsp.first.y(),uper_seg->get_v1().z()));

			}


		}
		else
		{
			qDebug() << "itsp contains more than 2 segment";
		}
	}
}

