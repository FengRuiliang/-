#include "supportor.h"

Supportor::Supportor()
{
	sup_points = new std::vector<Vec3f>;
	hatchs = new std::map<int, std::vector<std::pair<ivec2, ivec2>>>;
	polylines = new std::vector<std::vector<Vec3f>>;
}


Supportor::~Supportor()
{
}


void Supportor::add_supportting_point_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts)
{

	ClipperLib::Clipper cliper;
	ClipperLib::ClipperOffset off;
	Path pattern;
	for (int i = 0; i < 18; i++)
	{
		pattern << IntPoint(384 * cos(i * 20), 384 * sin(i * 20));
	}
	Paths sup_paths;
	Paths under_paths;
	Paths upper_paths;
	Paths mink_sum;
	for (int i = 10; i < cnts->size(); i++)
	{

		under_paths.clear();
		sup_paths.clear();
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
		sup_paths.clear();
		cliper.Clear();
		cliper.AddPaths(under_paths, ptClip, true);
		cliper.AddPaths(upper_paths, ptSubject, true);
		cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
		if (sup_paths.size() == 0)
		{
			continue;
		}
		MinkowskiSum(pattern, under_paths, mink_sum, true);
		cliper.Clear();
		cliper.AddPaths(mink_sum, ptClip, true);
		cliper.AddPaths(upper_paths, ptSubject, true);
		cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
		if (sup_paths.size())
		{
			qDebug() << i << "slice need supported";
		}

		for (int j = 0; j < sup_paths.size(); j++)
		{
			bool is_on_minkowskisum_1 = false;
			/*		find the first segment whose first point is not need support*/
			for (int k = 0; k < sup_paths[j].size(); k++)
			{
				IntPoint p1 = sup_paths[j][k];
				for each(Path poly in mink_sum)
				{
					if (PointInPolygon(p1, poly) == -1)
					{
						is_on_minkowskisum_1 = true;
						break;
					}
				}
				sup_paths[j].push_back(sup_paths[j].front());
				sup_paths[j].erase(sup_paths[j].begin());
				if (is_on_minkowskisum_1)
				{
					break;
				}
			}
			std::vector<bool> is_need_supported(sup_paths[j].size(), true);
			for (int k = 0; k < sup_paths[j].size(); k++)
			{
				IntPoint p2 = sup_paths[j][k];
				for each(Path poly in mink_sum)
				{
					if (PointInPolygon(p2, poly) == -1)
					{
						is_need_supported[k] = false;
						break;
					}
				}
			}
			std::vector<Vec3f> polyline;
			if (is_need_supported.front())
			{
				sup_points->push_back(Vec3f(sup_paths[j].front().X / 1e3, sup_paths[j].front().Y / 1e3, i*0.09));
				for (auto iter = sup_paths[j].begin(); iter != sup_paths[j].end(); iter++)
				{
					polyline.push_back(Vec3f(iter->X / 1e3, iter->Y / 1e3, i*0.09));
				}
				polyline.push_back(Vec3f(sup_paths[j].front().X / 1e3, sup_paths[j].front().Y / 1e3, i*0.09));
				qDebug() << polyline.size();
				polylines->push_back(polyline);
				//add_supportting_point_for_polylines(polyline);
				polyline.clear();
			}
			else
			{
				for (int k = 1; k +1< is_need_supported.size(); k++)
				{
					if (!is_need_supported[k - 1] && !is_need_supported[k] && !is_need_supported[k + 1])
					{
						if (polyline.size())
						{
							polylines->push_back(polyline);
							qDebug() << polyline.size();
						}

						
						//add_supportting_point_for_polylines(polyline);
						polyline.clear();
					}
					else
					{
						polyline.push_back(Vec3f(sup_paths[j][k].X / 1e3, sup_paths[j][k].Y / 1e3, i*0.09));
					}
				}
			}

		}
	}
}

void Supportor::add_supportting_point_for_polylines(std::vector<Vec3f> poly)
{

	for (int i = 0; i < poly.size() - 1;)
	{
		float err = 0, length = 0;
		Vec3f A = poly[i], dir1, dir2;
		Vec3f P = poly[++i];
		length = (P - A).length();
		if (length >= 3.0)
		{
			int part = length / 3 + 1;
			for (int j = 1; j < part; j++)
			{
				sup_points->push_back(A + j*length / part*(P - A));
			}
		}
		else
		{
			int j = i - 1;
			while (++i < poly.size() - 1)
			{
				dir2 = poly[i] - A;
				dir2.normalize();
				for (; j < i; j++)
				{
					dir1 = poly[j + 1] - poly[j];
					dir1.normalize();
					if (dir1.dot(dir2) != 0)
					{
						err += pow((poly[j + 1] - poly[i]).length(), 3.0)*dir1.cross(dir2).length() / dir1.dot(dir2);
					}
				}
				if ((poly[i] - A).length() > 3.0 || err > 1.0)
				{
					break;//the current point does not meet the requirement
				}
			}
			sup_points->push_back(poly[--i]);
		}
	}
}
