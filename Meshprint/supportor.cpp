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

void Supportor::add_support_point(std::vector<std::vector<std::vector<Segment>>> cnts)
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
				polyline << IntPoint(cnts[i][j][k].get_v1().x()*1e3,cnts[i][j][k].get_v1().y()*1e3);
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

void Supportor::sample_support_point(std::vector<std::vector<Segment>> uper, std::vector<std::vector<Segment>> under)
{
	
	
}

