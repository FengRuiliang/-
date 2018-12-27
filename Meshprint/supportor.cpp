#include "supportor.h"
#include <omp.h>

Supportor::Supportor()
{
	
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
	bool is_uniform = false;
	polylines = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	hatchs = new std::vector<std::vector<Segment>>(cnts->size());
	minkowskisums = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	sup_points = new std::vector<std::vector<Vec3f>>(cnts->size());
	ClipperLib::Clipper cliper;
	ClipperLib::ClipperOffset off;
	Path pattern;
	Hatch hatch_;
	for (int i = 0; i < 18; i++)
	{
		pattern << IntPoint(PBG * cos(i * 20), PBG * sin(i * 20));
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
		off.Execute(regions[i], -OSD);
		//regions[i] = tem;
	}


	for (int i=0;i<18;i++)
	{
		circle_<< IntPoint(500 * cos(i * 20), 500 * sin(i * 20));
	}
	for (int i = regions.size()-1; i >2 ; i--)
	{
		sup_paths.clear();
		
		mink_sum.clear();
		upper_paths = regions[i];
		under_paths = regions[i - 1];

		Paths temp;
		off.Clear();
		off.AddPaths(under_paths, jtMiter, etClosedPolygon);
		off.Execute(temp, PBG);
		cliper.Clear();
		cliper.AddPaths(temp, ptClip, true);
		cliper.AddPaths(upper_paths, ptSubject, true);
		cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
		
		if (sup_paths.size())
		{
			mink_sum.clear();
			MinkowskiSum(pattern, under_paths, mink_sum, true);
			cliper.Clear();
			cliper.AddPaths(mink_sum, ptClip, true);
			cliper.Execute(ctUnion, under_paths, pftNonZero, pftNonZero);
			for (int j = 0; j < under_paths.size(); j++)
			{
				std::vector<Vec3f> path;
				for (int k = 0; k < under_paths[j].size(); k++)
				{
					path.push_back(Vec3f(under_paths[j][k].X / 1e3, under_paths[j][k].Y / 1e3, i*0.09));
				}
				minkowskisums->at(i).push_back(path);
			}
			cliper.Clear();
			cliper.AddPaths(under_paths, ptClip, true);
			cliper.AddPaths(upper_paths, ptSubject, true);
			cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
			if (is_uniform)
			{
				add_supportting_point_by_uniform(sup_paths, i);
			}
			else
			{
				for (int j = 0; j < upper_paths.size(); j++)
				{
					findpolyline(upper_paths[j], under_paths, i);
				}
//				cliper.Clear();
// 				Paths tem;
// 				for (int j = 0; j < sup_points->size(); j++)
// 				{
// 					Path tem1;
// 					for (int k=0;k<sup_points->at(j).size();k++)
// 					{
// 						for (IntPoint p:circle_)
// 						{
// 							tem1 << IntPoint(p.X + sup_points->at(j)[k].x()*1e3, p.Y + sup_points->at(j)[k].y()*1e3);
// 						}
// 					}
// 					tem<<tem1;
// 				}
// 				//cliper.AddPaths(tem, ptClip, true);
// 				cliper.AddPaths(mink_sum, ptClip, true);
// 				cliper.Execute(ctUnion, under_paths, pftNonZero, pftNonZero);
				//for (int j = 0; j < 1; j++)
				//{
				//	// 				off.Clear();
				//	// 				off.AddPaths(upper_paths, jtMiter, etClosedPolygon);
				//	// 				off.Execute(upper_paths, -j * 420);
				//	cliper.Clear();
				//	cliper.AddPaths(under_paths, ptClip, true);
				//	cliper.AddPaths(upper_paths, ptSubject, true);
				//	cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
				//	if (sup_paths.size())
				//	{
				//		
				//	}
				//}
			}

			continue;
			CleanPolygons(sup_paths);
			hatch_.do_hatch_for_contour(sup_paths, hatchs->at(i), i*0.09);
			add_supportting_point_for_hatchs(hatchs->at(i),i);
		}
	}
	//merge();
}
inline void Supportor::findpolyline(Path target_paths, Paths mink_sum,int sliceid)
{
	
	std::vector<bool> is_in_minkows(target_paths.size());
	int num = 0;
	for (int i=0;i<target_paths.size();i++)
	{
		int count = 0;
		for each (Path poly_sum in mink_sum)
		{
			int re = PointInPolygon(target_paths[i], poly_sum);
			if (re == 1)
			{
				count++;
			}
			else if (re == -1)
			{
			
				is_in_minkows[i] = true;
				break;
			}
			if (count % 2==1)
			{
				is_in_minkows[i] = true;
			}
		}
		if (!is_in_minkows[i])
		{
			num++;
		}
	}
	if (num==target_paths.size())
	{
		std::vector<Vec3f> polyline;
		//polyline.push_back(Vec3f(target_paths.front().X / 1e3, target_paths.front().Y / 1e3, sliceid*0.09));
		for (int i=0;i<target_paths.size();i++)
		{
			polyline.push_back(Vec3f(target_paths[i].X / 1e3, target_paths[i].Y / 1e3, sliceid*0.09));
		}
		polyline.push_back(Vec3f(target_paths.front().X / 1e3, target_paths.front().Y / 1e3, sliceid*0.09));
		polylines->at(sliceid).push_back(polyline);
		sup_points->at(sliceid).push_back(Vec3f(target_paths.front().X / 1e3, target_paths.front().Y / 1e3, sliceid*0.09));
		add_supportting_point_for_polyline(polyline, sliceid);
	}
	else if(num!=0)
	{
		int cur_id=0;
		while (!is_in_minkows[cur_id])
		{
			cur_id = (cur_id + 1) % target_paths.size();
		}
		while (is_in_minkows[cur_id])
		{
			cur_id = (cur_id + 1) % target_paths.size();
		}
		
		int sta_id = cur_id;
		do 
		{
			std::vector<Vec3f> polyline;
			int last_id = (cur_id - 1 + target_paths.size())%target_paths.size();
			polyline.push_back(Vec3f(target_paths[last_id].X / 1e3, target_paths[last_id].Y / 1e3, sliceid*0.09));
			while (!is_in_minkows[cur_id])
			{
				//qDebug() << "out"<<cur_id;
				polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, sliceid*0.09));
				cur_id = (cur_id + 1) % target_paths.size();
			}
			polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, sliceid*0.09));
			if (polyline.size()>2)
			{
				polylines->at(sliceid).push_back(polyline);
				add_supportting_point_for_polyline(polyline, sliceid);
			}
			//qDebug() << "in" << cur_id;
			if (cur_id != sta_id)
			{
				cur_id = (cur_id + 1) % target_paths.size();

			}
		} while (cur_id!=sta_id);
	}
}

void Supportor::add_supportting_point_for_polyline(std::vector<Vec3f> poly,int sliceid)
{
	qDebug() << sliceid<<"whole number" << poly.size();
	int start_id = 0;
	do
	{
		Vec3f A = poly[start_id];
		int end_id = start_id;
		Vec3f dir1, dir2;
		float err1 = 0, err2 = 0, err3 = 0;
		//qDebug() << "start at" << start_id;
		while (++end_id < poly.size())
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
			if (err1 > PBL||err2 > PBL || err3 > ERR )
			{
				if (end_id- start_id==1)
				{
					float length = (poly[end_id] - poly[start_id]).length();
					Vec3f dir = poly[end_id] - poly[start_id];
					dir.normalize();
					int num = 0;
					for (; num*PBL < length; num++);
					float len = length / num;
					for (int j=1;j<num;j++)
					{
						sup_points->at(sliceid).push_back(poly[start_id] + j*len*dir);
					}
				}
				else 
				{
					end_id--;
				}
				if (end_id != poly.size() - 1)
				{
					sup_points->at(sliceid).push_back(poly[end_id]);
				}

				//qDebug() << "end at"<<end_id;
 				break;
			}
		}
		start_id = end_id;
	} while (start_id < poly.size() - 1);
}

void Supportor::merge()
{
	Path circle_;
	for (int i = 0; i < 18; i++)
	{
		circle_ << IntPoint(500 * cos(i * 20), 500 * sin(i * 20));
	}

	ClipperLib::Clipper cliper;
	for (int i=0;i<sup_points->size();i++)
	{
		for (int j=0;j<sup_points->at(i).size();j++)
		{
			Path tem;
			for (int k = 0; k < circle_.size(); k++)
			{
				tem << IntPoint(circle_[k].X + sup_points->at(i)[j].x()*1e3,
					circle_[k].Y + sup_points->at(i)[j].y()*1e3);
			}
			cliper.AddPath(tem, ptClip, true);
		}
	
	}
	Paths region_;
	cliper.Execute(ctUnion, region_, pftNonZero, pftNonZero);
}

 
void Supportor::add_supportting_point_for_hatchs(std::vector<Segment> hatchs,int sliceid)
{
	for (int i=0;i<hatchs.size();i++)
	{
		for (int j = 1; j*PBL < hatchs[i].get_length(); j++)
		{
			sup_points->at(sliceid).push_back(hatchs[i].get_v1() + j*PBL*hatchs[i].get_normal());
		}
	}
}

void Supportor::add_supportting_point_by_uniform(Paths poly,int sliceid)
{
	int distance = 1000;
	int max_x = -1000000;
	int min_x = 1000000;
	int max_y = -1000000;
	int min_y = 1000000;
	for (Path tar_path :poly)
	{
		for (IntPoint tar_p:tar_path)
		{
			max_x = tar_p.X > max_x ? tar_p.X: max_x;
			min_x = tar_p.X < min_x ? tar_p.X: min_x;
			max_y = tar_p.Y > max_y ? tar_p.Y : max_y;
			min_y = tar_p.Y < min_y ? tar_p.Y : min_y;
		}
	}
	min_x = (min_x / distance - 1)*distance;
	max_x = (max_x / distance + 1) * distance;
	min_y = (min_y / distance - 1) * distance;
	max_y = (max_y / distance + 1) * distance;
	for (int x_=min_x;x_<max_x;x_+= distance)
	{
		for (int y_=min_y;y_<max_y;y_+= distance)
		{
			int cross_num=0;
			for (Path tar_poly:poly)
			{
				if (PointInPolygon(IntPoint(x_, y_), tar_poly))
				{
					cross_num++;
				}
			}
			if (cross_num%2==1)
			{
				sup_points->at(sliceid).push_back(Vec3f(x_ / 1e3, y_ / 1e3, sliceid*0.09));
			}
		}
	}
}