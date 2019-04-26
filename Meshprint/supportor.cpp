#include "supportor.h"
#include <omp.h>
#include <QMatrix4x4>
#include "HE_mesh/Vec.h"
Supportor::Supportor()
{
	for (int i = 0; i < 36; i++)
	{
		pattern << IntPoint(PBG * cos(i * 10), PBG * sin(i * 10));
	}
	sup_ribs = new std::vector<Rib>;
}


Supportor::~Supportor()
{
	delete sup_points;
	delete polylines;
	delete hatchs;
	delete minkowskisums;
	delete sup_ribs;
}

void Supportor::generatePointsAndRibs(std::vector<std::vector<std::vector<Segment*>>>* cnts)
{
	sup_points = new std::vector<std::vector<Vec3f>>(cnts->size());
	sup_paths = new std::vector<std::vector<std::vector<Segment>>>(cnts->size());// 
	
	std::vector<Paths> contour(cnts->size());
	for (int i = 0; i < cnts->size(); i++)
	{
		for each (std::vector<Segment*> var in cnts->at(i))
		{
			Path path;
			for each (Segment* ptr_seg in var)
			{
				path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			}
			contour[i] << path;
		}
	}
	for (int i = 1; i < cnts->size(); i++)
	{
		ClipperLib::ClipperOffset off;
		Paths resualt;
		off.AddPaths(contour[i - 1], jtMiter, etClosedPolygon);
		off.Execute(resualt, PBG);
		ClipperLib::Clipper cliper;
		cliper.AddPaths(resualt, ptClip, true);
		cliper.AddPaths(contour[i], ptSubject, true);
		cliper.Execute(ctDifference, resualt, pftNonZero, pftNonZero);
		if (resualt.size())
		{
			Paths mink_sum;
			MinkowskiSum(pattern, contour[i - 1], mink_sum, true);
			SimplifyPolygons(mink_sum);
			for each (std::vector<Segment*> var in cnts->at(i))
			{
				std::vector<Segment> polyline;//the end of polyline has been supported
				for (int j = 0; j < var.size(); j++)
				{
					if (var[j]->get_length() == 0)
					{
						continue;
					}
					IntPoint v1(var[j]->get_v1().x()*1e3, var[j]->get_v1().y()*1e3);
					IntPoint v2(var[j]->get_v2().x()*1e3, var[j]->get_v2().y()*1e3);
					int count1 = 0, count2 = 0;
					for each (Path poly_sum in mink_sum)
					{
						count1 = PointInPolygon(v1, poly_sum) == 0 ? count1 : count1 + 1;
						count2 = PointInPolygon(v2, poly_sum) == 0 ? count2 : count2 + 1;
					}
					if (count1 % 2 != 0 && count2 % 2 == 0)
					{
						sup_points->at(i).push_back(var[j]->get_v2());
						polyline.push_back(*var[j]);
						link_to_ribs(var[j]->get_edge(),var[j]->get_v2());
					}
					else if (count1 % 2 == 0 && count2 % 2 == 0)
					{
						sup_points->at(i).push_back(var[j]->get_v2());
						link_to_ribs(var[j]->get_edge(), var[j]->get_v2());
						polyline.push_back(*var[j]);
					}
					else if (count1 % 2 == 0 && count2 % 2 != 0)
					{
						polyline.push_back(*var[j]);
						sup_paths->at(i).push_back(polyline);
						polyline.clear();
					}
					
				}
				if (!polyline.empty())
				{
					sup_paths->at(i).push_back(polyline);
				}
			}
		}
	}
	qDebug() << "finish add support point and ribs";
}
void Supportor::link_to_ribs(HE_edge* edge_,  Vec3f node_)
{
	bool finded_ = false;
	for (int i = 0; i < sup_ribs->size(); i++)
	{
		if (sup_ribs->at(i).edge_ == edge_)
		{
			finded_ = true;
			sup_ribs->at(i).nodes.push_back(node_);
		}
		else if (sup_ribs->at(i).edge_->pvert_==edge_->start_)
		{
			finded_ = true;
			sup_ribs->at(i).nodes.push_back(node_);
			sup_ribs->at(i).edge_ = edge_;
		}
	}
	if (finded_==false)
	{
		Rib rib;
		rib.edge_ = edge_;
		rib.nodes.push_back(node_);
		sup_ribs->push_back(rib);
	}
}

void Supportor::add_supportting_rib_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts)
{
	sup_points = new std::vector<std::vector<Vec3f>>(cnts->size());
	sup_segments = new std::vector<std::vector<Segment *>>(cnts->size());
	hatchs = new std::vector<std::vector<Segment>>(cnts->size());
	std::vector<Paths> contour(cnts->size());

	for (int i = 0; i < cnts->size(); i++)
	{
		for each (std::vector<Segment*> var in cnts->at(i))
		{
			Path path;
			for each (Segment* ptr_seg in var)
			{
				path << IntPoint(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
			}
			contour[i] << path;
		}
	}
	for (int i = 1; i < cnts->size(); i++)
	{
		ClipperLib::ClipperOffset off;
		Paths resualt;
		off.AddPaths(contour[i - 1], jtMiter, etClosedPolygon);
		off.Execute(resualt, PBG);
		ClipperLib::Clipper cliper;
		cliper.AddPaths(resualt, ptClip, true);
		cliper.AddPaths(contour[i], ptSubject, true);
		cliper.Execute(ctDifference, resualt, pftNonZero, pftNonZero);
		if (resualt.size())
		{
			Paths mink_sum;
			MinkowskiSum(pattern, contour[i - 1], mink_sum, true);
			SimplifyPolygons(mink_sum);
			for each (std::vector<Segment*> var in cnts->at(i))
			{
				std::vector<Segment*> polyline;
				for (int j = 0; j < var.size(); j++)
				{
					if (var[j]->get_length()==0)
					{
						continue;
					}
					IntPoint v1(var[j]->get_v1().x()*1e3, var[j]->get_v1().y()*1e3);
					IntPoint v2(var[j]->get_v2().x()*1e3, var[j]->get_v2().y()*1e3);
					int count1 = 0, count2 = 0;
					for each (Path poly_sum in mink_sum)
					{
						count1 = PointInPolygon(v1, poly_sum) == 0 ? count1 : count1 + 1;
						count2 = PointInPolygon(v2, poly_sum) == 0 ? count2 : count2 + 1;
					}
					if (count1 % 2 == 0 && count2 % 2 == 0)
					{
						sup_segments->at(i).push_back(var[j]);
						polyline.push_back(var[j]);
					}
					else
					{
						if (!polyline.empty())
						{
							add_supportting_point_for_overhangsegments(polyline, i);
							polyline.clear();
						}
					}
				}
				if (!polyline.empty())
				{
					add_supportting_point_for_overhangsegments(polyline, i);
				}
			}
		}
		//add_supportting_point_for_hatchs(resualt,i);
	}
}
void Supportor::add_supportting_point_for_overhangsegments(std::vector<Segment*> segments, int sliceid)
{
	qDebug() << ERR;
	sup_points->at(sliceid).push_back(segments.front()->get_v1() - Vec3f(0, 0, 10 * thickness_));
	for (int i=0;i<segments.size();i++)
	{
		float err3 = 0;
		Vec3f A = segments[i]->get_v1();
		for (int j=i;j<segments.size();j++)
		{
			Vec3f line = segments[j]->get_v2() - segments[i]->get_v1();
			line.normalize();
			float len = segments[j]->get_length();
			Vec3f nor = segments[j]->get_normal();
			if (line.dot(nor) != 0)
			{
				float tan = line.cross(nor).length() / line.dot(nor);
				err3 += pow(len, 3.0)*pow(tan, 2.0);
				err3 = sqrtf(err3 / 3);
			}
			if (err3>ERR)
			{
				sup_points->at(sliceid).push_back(segments[j]->get_v1()-Vec3f(0,0,10*thickness_));
				i = j;
				break;
			}
		}
	}
	sup_points->at(sliceid).push_back(segments.back()->get_v2() - Vec3f(0, 0, 10 * thickness_));
}


void Supportor::add_supportting_point_for_contours(std::vector<std::vector<std::vector<Segment*>>>* cnts)
{
	sup_points = new std::vector<std::vector<Vec3f>>(cnts->size());
	sup_lines = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	qDebug() << "PBG" << PBG;
	bool is_uniform = false;
	polylines = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());
	hatchs = new std::vector<std::vector<Segment>>(cnts->size());
	minkowskisums = new std::vector<std::vector<std::vector<Vec3f>>>(cnts->size());

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
	for (int i = 0; i < cnts->size(); i++)
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
	for (int i = 0; i < 18; i++)
	{
		circle_ << IntPoint(500 * cos(i * 20), 500 * sin(i * 20));
	}



	for (int i = regions.size() - 1; i > 0; i--)
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
			 //			hatch_.do_hatch_for_contour(sup_paths, hatchs->at(i), i*0.09,i);
			// 			add_supportting_point_for_hatchs(hatchs->at(i),i);
		}
	}
	//merge(cnts->size());
}

inline void Supportor::findpolyline(Path target_paths, Paths mink_sum, int sliceid)
{

	std::vector<bool> is_in_minkows(target_paths.size());
	int num = 0;
	for (int i = 0; i < target_paths.size(); i++)
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
			if (count % 2 == 1)
			{
				is_in_minkows[i] = true;
			}
		}
		if (!is_in_minkows[i])
		{
			num++;
		}
	}
	if (num == target_paths.size())
	{
		std::vector<Vec3f> polyline;
		std::vector<Vec2f> circle;
		for (int i = 0; i < target_paths.size(); i++)
		{
			polyline.push_back(Vec3f(target_paths[i].X / 1e3, target_paths[i].Y / 1e3, sliceid*0.09));
			circle.push_back(Vec2f(target_paths[i].X / 1e3, target_paths[i].Y / 1e3));
		}
		polyline.push_back(Vec3f(target_paths.front().X / 1e3, target_paths.front().Y / 1e3, sliceid*0.09));
		polylines->at(sliceid).push_back(polyline);

		auto minidisc = MINIDISC::miniDisc(circle);	
		if (minidisc.second > 1)
		{
			add_supportting_point_for_polyline(polyline, sliceid);
		}
		else
		{
			sup_points->at(sliceid).push_back(Vec3f(minidisc.first.x(), minidisc.first.y(), sliceid*thickness_));
		}

	}
	else if (num != 0)
	{
		int cur_id = 0;
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
			int last_id = (cur_id - 1 + target_paths.size()) % target_paths.size();
			polyline.push_back(Vec3f(target_paths[last_id].X / 1e3, target_paths[last_id].Y / 1e3, sliceid*0.09));
			while (!is_in_minkows[cur_id])
			{
				polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, sliceid*0.09));
				cur_id = (cur_id + 1) % target_paths.size();
			}
			polyline.push_back(Vec3f(target_paths[cur_id].X / 1e3, target_paths[cur_id].Y / 1e3, sliceid*0.09));
			if (polyline.size() > 2)
			{
				polylines->at(sliceid).push_back(polyline);
				add_supportting_point_for_polyline(polyline, sliceid);
			}
			if (cur_id != sta_id)
			{
				cur_id = (cur_id + 1) % target_paths.size();

			}
		} while (cur_id != sta_id);
	}
}

void Supportor::add_supportting_point_for_polyline(std::vector<Vec3f> polyin, int sliceid, float err)
{
	std::vector<Vec3f> P;
	int start_id = 0;
	do
	{
		Vec3f A = polyin[start_id];
		int end_id = start_id;
		Vec3f dir1, dir2;
		float sumlenth;
		float err1 = 0, err2 = 0, err3 = 0;
		//qDebug() << "start at" << start_id;
		while (++end_id < polyin.size())
		{
			err1 = (polyin[end_id] - polyin[end_id - 1]).length();
			err2 = (polyin[end_id] - polyin[start_id]).length();

			dir1 = polyin[end_id] - polyin[start_id];
			dir1.normalize();
			for (int j = start_id; j < end_id; j++)
			{
				dir2 = polyin[j + 1] - polyin[j];

				float length = dir2.length();
				dir2.normalize();
				if (dir1.dot(dir2) != 0)
				{
					err3 += pow(length, 3.0)*pow((dir1.cross(dir2).length() / dir1.dot(dir2)),2.0);
				}
				err3 = sqrtf(err3 / 3);
#ifdef MAXERROR
				err3 = std::max(abs(dir2.dot(dir1)), err3);
#endif
			}
			if (err1 > PBL || err2 > PBL || err3 > err)
			{
				if (end_id - start_id == 1)
				{
					float length = (polyin[end_id] - polyin[start_id]).length();
					Vec3f dir = polyin[end_id] - polyin[start_id];
					dir.normalize();
					int num = 0;
					for (; num*PBL < length; num++);
					float len = length / num;
					for (int j = 1; j < num; j++)
					{
						Vec3f p = polyin[start_id] + j*len*dir;
						p.z() = sliceid*thickness_;
						sup_points->at(sliceid).push_back(p);
					}
				}
				else
				{
					end_id--;
				}
				if (end_id != polyin.size() - 1)
				{
					Vec3f p = polyin[end_id];
					sup_points->at(sliceid).push_back(p);
					P.push_back(p);
				}
				break;
			}
		}
		start_id = end_id;
	} while (start_id < polyin.size() - 1);
	if (P.size()==1)
	{
		sup_points->at(sliceid).push_back((P[0]));
	}
	else
	{
		for (int i = 1; i < P.size(); i++)
		{
			if ((P[i] - P[i - 1]).length() > 1)
			{
				sup_points->at(sliceid).push_back(P[i]);
			}
			else
			{
				P[i] = P[i - 1];
			}
		}
	}
}

void Supportor::add_SP_for_rib(std::vector<Vec3f> polyin, std::vector<Vec3f>& polyout)
{
	int start_id = 0;
	do
	{
		Vec3f A = polyin[start_id];
		int end_id = start_id;
		Vec3f dir1, dir2;
		float sumlenth;
		float err1 = 0, err2 = 0, err3 = 0;
		while (++end_id < polyin.size())
		{
			err1 = (polyin[end_id] - polyin[end_id - 1]).length();
			err2 = (polyin[end_id] - polyin[start_id]).length();

			dir1 = polyin[end_id] - polyin[start_id];
			dir1.normalize();
			for (int j = start_id; j < end_id; j++)
			{
				dir2 = polyin[j + 1] - polyin[j];

				float length = dir2.length();
				sumlenth += length;
				dir2.normalize();
				if (dir1.dot(dir2) != 0)
				{
					err3 += pow(length, 3.0)*abs(dir1.cross(dir2).length() / dir1.dot(dir2)) / 3.f;
				}
			}
			if (err1 > PBL || err2 > PBL || err3 > GAP)
			{
				if (end_id - start_id == 1)
				{
					float length = (polyin[end_id] - polyin[start_id]).length();
					Vec3f dir = polyin[end_id] - polyin[start_id];
					dir.normalize();
					int num = 0;
					for (; num*PBL < length; num++);
					float len = length / num;
				}
				else
				{
					end_id--;
				}
				if (end_id != polyin.size() - 1)
				{
					polyout.push_back(polyin[end_id]);
				}
				break;
			}
		}
		start_id = end_id;
	} while (start_id < polyin.size() - 1);
}

void Supportor::merge(int num)
{
	std::vector<std::vector<Vec3f>> lines;
	for (int i=sup_points->size()-1;i>0 ;i--)
	{
		std::vector<Vec3f> points = sup_points->at(i);
		for (int k = 0; k < lines.size(); k++)
		{
			float dis = 1.2;
			float angle = 0.95;
			int id = -1;
			for (int j = 0; j < points.size(); j++)
			{
				Vec3f dir1 = points[j] - lines[k].back();
				Vec3f dir2 = lines[k].size()>1? (*lines[k].rbegin()) - *(lines[k].rbegin() + 1):dir1;
				dir1.z() = 0; dir2.z() = 0;
				float dis1 = dir1.length();
				dir1.normalize(); dir2.normalize();
				float angle1 = dir1.dot(dir2);
				if (dis1<dis&&angle1>angle)
				{
					dis = dis1;
					angle = angle1;
					id = j;
				}
			}
			if (id!=-1)
			{
				points.erase(points.begin() + id);
			}
			else
			{
				std::vector<Vec2f> in_;
				for (int m = 0; m < lines[k].size(); m++)
					in_.push_back(Vec2f(lines[k][m].x(), lines[k][m].y()));
				std::pair<Vec2f,float> c= MINIDISC::miniDisc(in_);
				if (c.second<1.01)
				{
					std::vector<Vec3f> t(1, Vec3f(c.first.x(), c.first.y(), lines[k].front().z()));
					sup_lines->at(i).push_back(t);
				}
				else
				{

					add_SP_for_rib(lines[k],sup_points->at(i - 1));

					sup_lines->at(i).push_back(lines[k]);
				}
				sup_lines->at(i).push_back(lines[k]);
				lines.erase(lines.begin() + k);
				k--;
			}
		}
		for (int j=0;j<sup_points->at(i).size();j++)
		{
			std::vector<Vec3f> line(1, sup_points->at(i)[j]);
			lines.push_back(line);
		}
	}
	for (int i=0;i<lines.size();i++)
	{
		sup_lines ->at(0).push_back(lines[i]);
	}

}

void Supportor::add_supportting_point_for_hatchs(Paths input,int sliceid)
{

	int gap = 420;
	std::map<int, std::vector<int>> lines_B;
	for (int i=0;i<input.size();i++)
	{
		for (int j=0;j<input[i].size();j++)
		{
			ivec2 p1(input[i][j].X, input[i][j].Y);
			ivec2 p2(input[i][(j + 1) % input[i].size()].X, input[i][(j + 1) % input[i].size()].Y);
			int y_min, y_max;
			if (p1.y() < p2.y())
			{
				y_min = p1.y();
				y_max = p2.y();
			}
			else if (p1.y() > p2.y())
			{
				y_max = p1.y();
				y_min = p2.y();
			}
			else
			{
				continue;
			}
			int id = y_min / gap - 2;
			while (id*gap < y_min)
			{
				id++;
			}
			for (; id*gap < y_max; id++)
			{
				int y_ = id*gap;
				int x_ = p1.x() + (p2.x() - p1.x())*(id*gap - p1.y()) / (p2.y() - p1.y());
				lines_B[y_].push_back(x_);
			}

		}
	}
	for (auto iter = lines_B.begin(); iter != lines_B.end(); iter++)
	{
		std::sort(iter->second.begin(), iter->second.end());
		for (int j = 0; j < iter->second.size() - 1; j++)
		{
			int x1 = iter->second[j];
			int x2 = iter->second[++j];
			int y = iter->first;
			Vec3f v1(x1 *1e-3, y * 1e-3, sliceid*thickness_);
			Vec3f v2(x2*1e-3, y*1e-3, sliceid*thickness_);
			hatchs->at(sliceid).push_back(Segment(v1, v2));
			for (int k=1;k*10000+x1<x2;k++)
			{
				Vec3f v(k * 10 + x1*1e-3, y*1e-3, (sliceid - 50)*thickness_);
				sup_points->at(sliceid).push_back(v);
			}
		}
	}

}

void Supportor::add_supportting_point_by_uniform(Paths poly, int sliceid)
{
	int distance = 1000;
	int max_x = -1000000;
	int min_x = 1000000;
	int max_y = -1000000;
	int min_y = 1000000;
	for (Path tar_path : poly)
	{
		for (IntPoint tar_p : tar_path)
		{
			max_x = tar_p.X > max_x ? tar_p.X : max_x;
			min_x = tar_p.X < min_x ? tar_p.X : min_x;
			max_y = tar_p.Y > max_y ? tar_p.Y : max_y;
			min_y = tar_p.Y < min_y ? tar_p.Y : min_y;
		}
	}
	min_x = (min_x / distance - 1)*distance;
	max_x = (max_x / distance + 1) * distance;
	min_y = (min_y / distance - 1) * distance;
	max_y = (max_y / distance + 1) * distance;
	for (int x_ = min_x; x_ < max_x; x_ += distance)
	{
		for (int y_ = min_y; y_ < max_y; y_ += distance)
		{
			int cross_num = 0;
			for (Path tar_poly : poly)
			{
				if (PointInPolygon(IntPoint(x_, y_), tar_poly))
				{
					cross_num++;
				}
			}
			if (cross_num % 2 == 1)
			{
				sup_points->at(sliceid).push_back(Vec3f(x_ / 1e3, y_ / 1e3, sliceid*0.09));
			}
		}
	}
}

void Supportor::buildTreeStructure()
{
	struct {
		bool operator()(Vec3f a, Vec3f b) const
		{
			return a.z() < b.z();
		}

	} customLess;

	std::vector<Vec3f> P;
	for (int i = 0; i < sup_points->size(); i++)
	{
		for (int j = 0; jtMiter < sup_points->at(i).size(); j++)
		{
			P.push_back(sup_points->at(i)[j]);
		}
	}
	std::sort(P.begin(), P.end(), customLess);
	while (!P.empty())
	{
		Vec3f p1 = P.back();
		P.pop_back();
		for (int i = 0; i < P.size(); i++)
		{
			Vec3f p2 = P[i];
			Vec3f hypotenuse = p2 - p1;
			hypotenuse.normalize();
			float theta1_radian = hypotenuse.dot(Vec3f(0, 0, 1));
			Vec3f axis = (p2 - p1).cross(Vec3f(0, 0, 1));
			axis.normalize();

		}
	}
}

Vec3f Supportor::computerIntersection(Paths& mink_sum, Segment* ptr_seg)
{
	IntPoint v1(ptr_seg->get_v1().x()*1e3, ptr_seg->get_v1().y()*1e3);
	IntPoint v2(ptr_seg->get_v2().x()*1e3, ptr_seg->get_v2().y()*1e3);
	for each (Path circle in mink_sum)
	{
		int count1 = 0, count2 = 0;
		if (PointInPolygon(v1, circle) != 0 && PointInPolygon(v2, circle) == 0)
		{
			for (int i = 0; i < circle.size(); i++)
			{
				ivec3 A(v1.X, v1.Y,0);
				ivec3 B(v2.X, v2.Y,0);
				ivec3 C(circle[i].X, circle[i].Y,0);
				ivec3 D(circle[(i + 1) % circle.size()].X, circle[(i + 1) % circle.size()].Y,0);

				if ((C - A).cross(B - A).z()*(D - A).cross(B - A).z() < 0 &&
					(A - C).cross(D - C).z()*(B - C).cross(D - C).z() < 0)

				{
					float	t=abs((C - A).cross(B - A).z())/
						(abs((C - A).cross(B - A).z()) + abs((D - A).cross(B - A).z()));
					return ptr_seg->get_v1() + t*(ptr_seg->get_v2() - ptr_seg->get_v1());
					break;
				}
			}
		}
	}
}

