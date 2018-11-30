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

	ClipperLib::Clipper cliper;
	ClipperLib::ClipperOffset off;
	Path pattern;
	for (int i = 0; i < 18; i++)
	{
		pattern << IntPoint(384 * cos(i * 20), 384 * sin(i * 20));
	}
	Paths solution;
	Paths under_paths;
	Paths upper_paths;
	Paths mink_sum;
	for (int i = 2; i < cnts->size(); i++)
	{
		under_paths.clear();
		solution.clear();
		upper_paths.clear();
		mink_sum.clear();
		for each (std::vector<Segment*> var in cnts->at(i - 1))
		{
			Path path;
			for each (Segment* ptr_seg in var)
			{
				path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			}
			under_paths << path;
		}
		
		for each (std::vector<Segment*> var in cnts->at(i))
		{
			Path path;
			for each (Segment* ptr_seg in var)
			{
				path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			}
			upper_paths << path;
		}
		off.Clear();
		off.AddPaths(under_paths, jtSquare, etClosedPolygon);
		off.Execute(solution, 384);
		
		cliper.Clear();
		cliper.AddPaths(solution, ptClip, true);
		cliper.AddPaths(upper_paths, ptSubject, true);
		cliper.Execute(ctDifference, solution, pftNonZero, pftNonZero);
		if (solution.size())
		{
			qDebug() << "jump into sampling ";
			MinkowskiSum(pattern, under_paths, mink_sum, true);
			solution.clear();
			cliper.Clear();
			cliper.AddPaths(mink_sum, ptClip, true);
			cliper.AddPaths(under_paths, ptSubject, true);
			cliper.Execute(ctUnion, mink_sum, pftNonZero, pftNonZero);
			cliper.Clear();
			cliper.AddPaths(mink_sum, ptClip, true);
			cliper.AddPaths(under_paths, ptSubject, true);
			cliper.Execute(ctDifference, solution, pftNonZero, pftNonZero);
			for (int j = 0; j < solution.size(); j++)
			{
				for (int k = 0; k < solution[j].size(); k++)
				{
					IntPoint p1 = solution[j][k];
					IntPoint p2 = solution[j][(k + 1) % solution[j].size()];
					bool is_on_minkowskisum_1 = false;
					bool is_on_minkowskisum_2 = false;
					for each(Path poly in mink_sum)
					{
						if (PointInPolygon(p1, poly) == -1)
						{
							is_on_minkowskisum_1 = true;
						}
						if (PointInPolygon(p2, poly) == -1)
						{
							is_on_minkowskisum_2 = true;
						}
					}
					if (!is_on_minkowskisum_2)
					{
						if (!is_on_minkowskisum_1)
						{
							Vec3f p = Vec3f(p1.X*1e-3, p1.Y*1e-3, i*0.09);
							sup_points->push_back(p);
						}
						Vec3f pf(p1.X*1e-3, p1.Y*1e-3, i*0.09);
						Vec3f pb(p2.X*1e-3, p2.Y*1e-3, i*0.09);
						Vec3f vec = pb - pf;
						float length_left = vec.length();
						vec.normalize();
						while (length_left > 3)
						{
							length_left -= 3;
							Vec3f p = pf + 3.0f*vec;
							sup_points->push_back(p);
						}
					}
				}
			}
		}
	}
}