#include "supportor.h"
#include <omp.h>
#include <QMatrix4x4>
Supportor::Supportor()
{
	sup_lines = new std::vector<std::vector<Vec3f>>;
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
	qDebug() << "PBG" << PBG;
	bool is_uniform = false;
	polylines = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	hatchs = new std::vector<std::vector<Segment>>(cnts->size());
	minkowskisums = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	sup_points = new std::vector<std::vector<Vec3f>>(cnts->size());
	sup_rectangle = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	ClipperLib::Clipper cliper;
	ClipperLib::ClipperOffset off;
	Path pattern;
	Hatch hatch_;
	for (int i = 0; i < 36; i++)
	{
		pattern << IntPoint(PBG * cos(i * 10), PBG * sin(i * 10));
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
		CleanPolygons(regions[i]);
		SimplifyPolygons(regions[i]);
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
			}
			
// 			off.Clear();
// 			off.AddPaths(upper_paths, jtMiter, etClosedPolygon);
// 			off.Execute(upper_paths, -2 * OSD);
// 			cliper.Clear();
// 			cliper.AddPaths(under_paths, ptClip, true);
// 			cliper.AddPaths(upper_paths, ptSubject, true);
// 			cliper.Execute(ctDifference, sup_paths, pftNonZero, pftNonZero);
// 			CleanPolygons(sup_paths);
// 			hatch_.do_hatch_for_contour(sup_paths, hatchs->at(i), i*0.09,i);
// 			add_supportting_point_for_hatchs(hatchs->at(i),i);
		}
	}
	//merge(cnts->size());
	qDebug() << "finish";
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

void Supportor::merge(int num)
{
	
	for (int i=0;i<sup_points->size();i++)
	{
		for (int j=0;j<sup_points->at(i).size();j++)
		{
			Vec3f point = sup_points->at(i)[j];
			int is_attach_for_ith_line = -1;
			for (int k = 0; k < sup_lines->size(); k++)
			{
				if (sup_lines->at(k).size()>1)
				{
					auto riter = sup_lines->at(k).rbegin();
					Vec3f dir = *riter - *(riter+1); 
					dir.normalize();
					Vec3f dir1 = point - *riter;
					float length = dir1.length();
					
					dir1.normalize();
					float angle = dir1.dot(dir);
					if (length<0.9&&angle>.9)
					{
						is_attach_for_ith_line = k;
						break;
					}

				}
				else if (( point- sup_lines->at(k).back()).length() < 0.9)
				{
					is_attach_for_ith_line = k;
					break;
				}
			}
			if (is_attach_for_ith_line == -1)
			{
				std::vector<Vec3f> new_line(1, point);
				sup_lines->push_back(new_line);
			}
			else
			{
				sup_lines->at(is_attach_for_ith_line).push_back(point);
			}
		}
		sup_points->at(i).clear();
	}

	for (auto iter=sup_lines->begin();iter!=sup_lines->end();)
	{

		if (iter->size()<3)
		{
			int id = iter->front().z() / 0.09;
			sup_points->at(id).push_back(iter->front());
			iter=sup_lines->erase(iter);
		}
		else
		{
			float max_ = -1e6;
			for (Vec3f p:*iter)
			{
				max_ = max_ > p.z() ? max_ : p.z();
			}
// 			for (Vec3f p:*iter)
// 			{
// 				p.z() = max_;
// 			}
// 			
		
			Path for_extrude;
			for (int k = 0; k < iter->size(); k++)
			{
				for_extrude << IntPoint(iter->at(k).x()*1e3, iter->at(k).y()*1e3);
				/*iter->at(k).z() -= 0.9;*/
			}
			ClipperLib::ClipperOffset off;
			off.AddPath(for_extrude, jtMiter, etOpenButt);
			Paths solution;
			off.Execute(solution, 300);
			std::vector<Vec3f> line1;
			for (auto iter1 = solution.begin(); iter1 != solution.end(); iter1++)
			{
				for (IntPoint p : *iter1)
				{
					line1.push_back(Vec3f(p.X / 1e3, p.Y / 1e3, max_));
				}
			}
			int id = max_ / 0.09 - 10;
			sup_rectangle->at(max_/.09).push_back(line1);
			sup_points->at(id).push_back(iter->front());
			sup_points->at(id).push_back(iter->back());
			add_supportting_point_for_polyline(*iter, id);
			++iter;
		}
	}
}

 
void Supportor::add_supportting_point_for_hatchs(std::vector<Segment> hatchs,int sliceid)
{
	Path pattern;
	std::vector<std::vector<Vec3f>> lines;

	for (int i=0;i<hatchs.size();i++)
	{
		int num_part = 0;
		for (; num_part*PBL < hatchs[i].get_length(); num_part++);
		float len_per = hatchs[i].get_length() / num_part;
		len_per = PBL;
		for (int j = 1; j*len_per < hatchs[i].get_length()-1e-3; j++)
		{
			Vec3f tem = hatchs[i].get_v1() + j*len_per*hatchs[i].get_normal();
			int is_attach_for_ith_line = -1;
			for (int k = 0; k < lines.size(); k++)
			{
				if ((tem - lines[k].back()).length() < 0.942)
				{
					is_attach_for_ith_line = k;
					break;
				}
			}
			if (is_attach_for_ith_line==-1)
			{
				std::vector<Vec3f> new_line(1,tem);
				lines.push_back(new_line);
			}
			else
			{
				lines[is_attach_for_ith_line].push_back(tem);
			}
			
		}
	}

	for (auto iter = lines.begin(); iter!=lines.end();iter++)
	{
		if (iter->size() == 1)
		{
			sup_points->at(sliceid).push_back(iter->front());
			//iter=lines->at(sliceid).erase(iter);
		}
		else
		{
			
			sup_points->at(sliceid - 50).push_back(iter->front() - Vec3f(0, 0, 0.9));
			sup_points->at(sliceid - 50).push_back(iter->back() - Vec3f(0, 0, 0.9));
			//sup_lines->at(sliceid).push_back(*iter);
			
			Path for_extrude;
			for (int k=0;k<iter->size();k++)
			{
				for_extrude << IntPoint(iter->at(k).x()*1e3, iter->at(k).y()*1e3);
				iter->at(k).z() -= 0.9;
			}
			add_supportting_point_for_polyline(*iter, sliceid - 10);
			ClipperLib::ClipperOffset off;
			off.AddPath(for_extrude, jtMiter, etOpenButt);
			Paths solution;
			off.Execute(solution, 300);
			std::vector<Vec3f> line1;
			for (auto iter1 = solution.begin(); iter1 != solution.end(); iter1++)
			{
				for (IntPoint p:*iter1)
				{
					line1.push_back(Vec3f(p.X / 1e3, p.Y / 1e3, sliceid*0.09));
				}
			}
			sup_rectangle->at(sliceid).push_back(line1);
			continue;
// 			Paths mink_sum;
// 			MinkowskiSum(pattern, for_extrude, mink_sum, false);
// 			//qDebug() << for_extrude.size()<<mink_sum.size();
// 
// 			for (int j = 0; j < mink_sum.size(); j++)
// 			{
// 				std::vector<Vec3f> linet;
// 				for (int k = 0; k < mink_sum[j].size(); k++)
// 				{
// 					linet.push_back(Vec3f(mink_sum[j][k].X / 1e3, mink_sum[j][k].Y / 1e3, sliceid*0.09));
// 				}
// 				sup_lines->at(sliceid).push_back(linet);
// 			}
// 			sup_lines->at(sliceid).push_back(*iter);
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



void Supportor::buildTreeStructure()
{

}
