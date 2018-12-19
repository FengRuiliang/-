#include "supportor.h"
#include <omp.h>

Supportor::Supportor()
{
	sup_points = new std::vector<Vec3f>;
}


Supportor::~Supportor()
{
	delete sup_points;
	delete polylines;
	delete hatchs;
	delete minkowskisums;
}


void Supportor::add_supportting_point_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts)
{
	polylines = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	hatchs = new std::vector<std::vector<Segment>>(cnts->size());
	minkowskisums = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	ClipperLib::Clipper cliper;
	ClipperLib::ClipperOffset off;
	Path pattern;
	Hatch hatch_;
	for (int i = 0; i < 18; i++)
	{
		pattern << IntPoint(384 * cos(i * 20), 384 * sin(i * 20));
	}
	Paths sup_paths;
	Paths under_paths;
	Paths upper_paths;
	Paths mink_sum;
	Path circle_;
	std::vector<Paths> regions(cnts->size());
	for (int i=0;i<cnts->size();i++ )
	{
		Paths tem;
		for each (std::vector<Segment*> var in cnts->at(i))
		{
			Path path;
			for each (Segment* ptr_seg in var)
			{
				path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			}
			tem << path;
		}
		off.Clear();
		off.AddPaths(tem, jtMiter, etClosedPolygon);
		off.Execute(regions[i], -384);
	}


	for (int i=0;i<18;i++)
	{
		circle_<< IntPoint(500 * cos(i * 20), 500 * sin(i * 20));
	}
	for (int i = regions.size()-1; i >2 ; i--)
	{
	
		sup_paths.clear();
		
		mink_sum.clear();
		//for each (std::vector<Segment*> var in cnts->at(i - 1))
		//{
		//	Path path;
		//	for each (Segment* ptr_seg in var)
		//	{
		//		path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
		//	}
		//	under_paths << path;
		//}
		//for each (std::vector<Segment*> var in cnts->at(i))
		//{
		//	Path path;
		//	for each (Segment* ptr_seg in var)
		//	{
		//		path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
		//	}
		//	upper_paths << path;
		//}

		upper_paths = regions[i];
		under_paths = regions[i - 1];

		Paths temp;
		off.Clear();
		off.AddPaths(under_paths, jtMiter, etClosedPolygon);
		off.Execute(temp, 384);
		cliper.Clear();
		cliper.AddPaths(temp, ptClip, true);
		cliper.AddPaths(upper_paths, ptSubject, true);
		cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
		if (sup_paths.size())
		{
			mink_sum.clear();
			MinkowskiSum(pattern, under_paths, mink_sum, true);
			cliper.Clear();
			Paths tem(sup_points->size());
			for (int j=0;j<sup_points->size();j++)
			{
				for (int k=0;k<circle_.size();k++)
				{
					tem[j] <<IntPoint(circle_[k].X+sup_points->at(j).x()*1e3, circle_[k].Y + sup_points->at(j).y()*1e3);
				}
			}
			cliper.AddPaths(tem, ptClip, true);
			cliper.AddPaths(mink_sum, ptClip, true);
			cliper.Execute(ctUnion, under_paths, pftNonZero, pftNonZero);
			for (int j = 0; j < 1; j++)
			{
// 				off.Clear();
// 				off.AddPaths(upper_paths, jtMiter, etClosedPolygon);
// 				off.Execute(upper_paths, -j * 420);
				cliper.Clear();
				cliper.AddPaths(under_paths, ptClip, true);
				cliper.AddPaths(upper_paths, ptSubject, true);
				cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
				if (sup_paths.size())
				{
					for (int j = 0; j < upper_paths.size(); j++)
					{
						findpolyline(upper_paths[j], under_paths, i);
					}
				}
			}
			for (int j=0;j<sup_paths.size();j++)
			{
				std::vector<Vec3f> path;
				for (int k=0;k<sup_paths[j].size();k++)
				{
					path.push_back(Vec3f(sup_paths[j][k].X / 1e3, sup_paths[j][k].Y / 1e3, i*0.09));
					
				}
				minkowskisums->at(i).push_back(path);
			}
			//CleanPolygons(sup_paths);
			//hatch_.do_hatch_for_contour(sup_paths, hatchs->at(i), i*0.09);
			//add_supportting_point_for_hatchs(hatchs->at(i));
		}
	}
	//merge();
}

void Supportor::add_supportting_point_for_polyline(std::vector<Vec3f> poly)
{
	int start_id = 0;
	do
	{
		Vec3f A = poly[start_id];
		int end_id = start_id;
		Vec3f dir1, dir2;
		float err1 = 0, err2 = 0, err3 = 0;
		while (++end_id<poly.size())
		{
			err1 = (poly[end_id] - poly[end_id-1]).length();
			err2 = (poly[end_id] - poly[start_id]).length();

			dir1 = poly[end_id] - poly[end_id - 1];
			dir1.normalize();
			for (int j = start_id; j < end_id; j++)
			{
				dir2 = poly[end_id] - poly[start_id];
				dir2.normalize();
				if (dir1.dot(dir2) != 0)
					err3 += pow((poly[j + 1] - poly[j]).length(), 3.0)*abs(dir1.cross(dir2).length() / dir1.dot(dir2));
			}
			//qDebug() << err3;
			if (err1 > PBL||err2 > PBL || err3 > ERR )
			{
				if (end_id - start_id == 1)
				{
					int part = (poly[end_id] - poly[start_id]).length() / PBL + 1;
					float len = (poly[end_id] - poly[start_id]).length() / part;
					for (int j = 1; j <= part; j++)
					{
						sup_points->push_back(poly[start_id] + j*len*dir2);
					}
				}
				else
				{
					sup_points->push_back(poly[--end_id]);
				}
				break;
			}
		}
		start_id = end_id;
	} while (start_id < poly.size() - 1);
}

void Supportor::merge()
{
	std::vector<bool> merged(sup_points->size());
	for (int i=0;i<sup_points->size();i++)
	{
		for (int j=i+1;j<sup_points->size();j++)
		{
			float l = (sup_points->at(j) - sup_points->at(i)).length();
			if (l<1.0)
			{
				merged[j] = true;
			}
		}
	}
	std::vector<Vec3f> final_points;
	for (int i=0;i<merged.size();i++)
	{
		if (!merged[i])
		{
			final_points.push_back(sup_points->at(i));
		}
	}
	sup_points->clear();
	for (int i = 0; i<final_points.size(); i++)
	{
		sup_points->push_back(final_points[i]);
	}
}

inline void Supportor::findpolyline(Path target_paths, Paths mink_sum,int num)
{
	
	float threathod = 0.0f;
	int cur_id = 0;
	int sta_id = 0;
	bool is_on_minkowski = false;
	//找到第一个在minkowski上的点
	do
	{
		is_on_minkowski = false;
		for each (Path poly_sum in mink_sum)
		{
			is_on_minkowski = is_on_minkowski || PointInPolygon(target_paths[cur_id], poly_sum);
		}
		cur_id = (cur_id + 1) % target_paths.size();
	} while (cur_id != sta_id && !is_on_minkowski);
	// 未找到第一个在minkowski上的点，即全部都不在minkowski上
	if (cur_id == sta_id)
	{
		/*qDebug() << "the polyline is one circle";*/
		std::vector<Vec3f> polyline;
		sup_points->push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, num*0.09));
		polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, num*0.09));
		cur_id = (cur_id + 1) % target_paths.size();
		do
		{
			polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, num*0.09));
			cur_id = (cur_id + 1) % target_paths.size();
		} while (cur_id != sta_id);
		polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, num*0.09));
		polylines->at(num).push_back(polyline);
		add_supportting_point_for_polyline(polyline);

		
	}
	else
	{
		sta_id = cur_id;//set the first point, which is on the minkowski ,as the new start point
		do
		{
			//the current point is on  minkowski , try to find the first point which is not on minkowski
			IntPoint end_point;
			do
			{
				end_point = target_paths[cur_id];//record the point which is the first point in polyline
				is_on_minkowski = false;
				for each (Path poly_sum in mink_sum)
				{
					is_on_minkowski = is_on_minkowski || PointInPolygon(target_paths[cur_id], poly_sum);
				}
				cur_id = (cur_id + 1) % target_paths.size();
			} while (cur_id != sta_id && is_on_minkowski);
			if (cur_id != sta_id)
			{
				std::vector<Vec3f> polyline;
				//the current point is not on minkowski, start to record polyline
				polyline.push_back(Vec3f(end_point.X / 1e3, end_point.Y / 1e3, num*0.09));
				do
				{
					polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, num*0.09));
					
					is_on_minkowski = false;
					for each (Path poly_sum in mink_sum)
					{
						is_on_minkowski = is_on_minkowski || PointInPolygon(target_paths[cur_id], poly_sum);
					}
					cur_id = (cur_id + 1) % target_paths.size();
					end_point = target_paths[cur_id];//record the point which is the end point in polyline
				} while (cur_id != sta_id && !is_on_minkowski);
				//the current point is on minkowski
				polyline.push_back(Vec3f(end_point.X / 1e3, end_point.Y / 1e3, num*0.09));
				polylines->at(num).push_back(polyline);
				add_supportting_point_for_polyline(polyline);

			}
		} while (cur_id != sta_id);
	}
}
 
void Supportor::add_supportting_point_for_hatchs(std::vector<Segment> hatchs)
{
	for (int i=0;i<hatchs.size();i++)
	{
		int part = hatchs[i].get_length() / PBL + 1;
		float len = hatchs[i].get_length() / part;
		for (int j = 1; j < part; j++)
		{
			sup_points->push_back(hatchs[i].get_v1() + j*len*hatchs[i].get_normal());
		}
	}
}
