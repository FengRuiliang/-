#include "supportor.h"

Supportor::Supportor()
{
	sup_points = new std::vector<Vec3f>;
	hatchs = new std::map<int, std::vector<std::pair<ivec2, ivec2>>>;
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
	for (int i = 16; i < cnts->size(); i++)
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
			MinkowskiSum(pattern, under_paths, mink_sum, true);
			for (int m = 0; m < 1; m++)
			{
				off.Clear();
				off.AddPaths(upper_paths, jtMiter, etClosedPolygon);
				off.Execute(upper_paths, -250 * m);


				solution.clear();
				cliper.Clear();
				cliper.AddPaths(mink_sum, ptClip, true);
				cliper.AddPaths(upper_paths, ptSubject, true);
				cliper.Execute(ctDifference, solution, pftNonZero, pftNonZero);
				for (int j = 0; j < solution.size(); j++)
				{

					bool is_on_minkowskisum_1 = false;
					bool is_on_minkowskisum_2 = false;
					/*		find the first segment whose first point is not need support and second support is need support*/
					for (int k = 0; k < solution[j].size(); k++)
					{
						IntPoint p1 = solution[j][k];
						IntPoint p2 = solution[j][(k + 1) % solution[j].size()];
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
						solution[j].push_back(solution[j].front());
						solution[j].erase(solution[j].begin());
						if (is_on_minkowskisum_1 && !is_on_minkowskisum_2)
						{
							break;
						}
					}
					if (!is_on_minkowskisum_1)
					{
						sup_points->push_back(Vec3f(solution[j].front().X*1e-3, solution[j].front().Y*1e-3, i*0.09));
					}
					float dis = 0;
					IntPoint p1, p2;
					Vec3f v1, v2;
					Vec3f cur_dir;
					p1 = solution[j][0];
					p2 = solution[j][(0 + 1) % solution[j].size()];
					v1 = Vec3f(p1.X*1e-3, p1.Y*1e-3, i*0.09);
					v2 = Vec3f(p2.X*1e-3, p2.Y*1e-3, i*0.09);
					for (int k = 0; k < solution[j].size(); k++)
					{
						p1 = solution[j][k];
						p2 = solution[j][(k + 1) % solution[j].size()]; 
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
						if (!is_on_minkowskisum_1 || !is_on_minkowskisum_2)
						{
							v1 = Vec3f(p1.X*1e-3, p1.Y*1e-3, i*0.09);
							v2 = Vec3f(p2.X*1e-3, p2.Y*1e-3, i*0.09);
							cur_dir = v2 - v1;
							dis += cur_dir.length();
							cur_dir.normalize();
							while (dis > 3)
							{
								dis -= 3;
								sup_points->push_back(v2 - dis*cur_dir);
							}
						}
						else
						{
							dis = 0;
						}
					}
				}
			}
			Hatch minkows_hatch;
			(*hatchs)[i] = minkows_hatch.do_hatch_for_contour(solution);
			qDebug() << i;
		}
	}
}