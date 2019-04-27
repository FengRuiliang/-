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
}


Supportor::~Supportor()
{

}

void Supportor::generatePointsAndRibs(std::vector<std::vector<std::vector<Segment*>>>* cnts)
{
	sup_paths.resize(cnts->size());
	sup_nodes_.resize(cnts->size());
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
				std::vector<Segment> segments;//the end of polyline has been supported
				Polyline temppolyline;
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
						Polyline added;
						sup_paths[i].push_back(added);
						Node * node_ = new Node(var[j]);
						sup_paths[i].back().paths.push_back(node_);
						sup_nodes_[i].push_back(node_);
					}
					else if (count1 % 2 == 0 && count2 % 2 == 0)
					{
						Node * node_ = new Node(var[j]);
						sup_paths[i].back().paths.push_back(node_);
						sup_nodes_[i].push_back(node_);
					}
					else if (count1 % 2 == 0 && count2 % 2 != 0)
					{
						Node * node_ = new Node(var[j]);
						sup_paths[i].back().paths.push_back(node_);
					}	
				}
			}
			link_to_ribs(sup_nodes_[i]);
		}
	
	}

}
void Supportor::link_to_ribs(std::vector<Node*> nodes)
{

	for (int i = 0; i < ribs.size(); i++)
	{
		for (int j = 0; j < nodes.size(); j++)
		{
			if (ribs[i]->Edge() == nodes[j]->Edge() || ribs[i]->Edge()->pvert_ == nodes[j]->Edge()->start_)
			{
				if (ribs[i]->Normal().dot(nodes[j]->Normal()) > ribs[i]->Angle())
				{
					ribs[i]->setAngle(ribs[i]->Normal().dot(nodes[j]->Normal()));
					ribs[i]->setCandidateNode(nodes[j]);
				}
			}

		}
	}
	for (int i=0;i<ribs.size();i++)
	{
		if (ribs[i]->CandidateNode()!=NULL)
		{
			ribs[i]->setEdge(ribs[i]->CandidateNode()->Edge());
			ribs[i]->Nodes().push_back(ribs[i]->CandidateNode());
			ribs[i]->setAngle(-1);
			ribs[i]->Normal() = ribs[i]->CandidateNode()->Normal();
			ribs[i]->CandidateNode()->setRib(ribs[i]);
			ribs[i]->setCandidateNode(NULL);
		}
	}
	for (int j=0;j<nodes.size();j++)
	{
		if (nodes[j]->getRib()==NULL)
		{
			Rib* rib_=new Rib(nodes[j]);
			ribs.push_back(rib_);
		}
	}

}
void Supportor::opitimizePointsandRibs()
{
	for (int i = 0; i < sup_paths.size(); i++)
	{
		for (int j=0;j<sup_paths[i].size();j++)
		{
			std::vector<Node*>& polyin = sup_paths[i][j].paths;
			int start_id = 0;
			do
			{
			
				int end_id = start_id;
				Vec3f dir1, dir2;
				float err3 = 0;
				while (++end_id < polyin.size())
				{
					dir1 = polyin[end_id]->getPosition() -polyin[start_id]->getSegments()->get_v1() ;
					dir1.normalize();
					for (int j = start_id; j < end_id; j++)
					{
					
						dir2 = polyin[j]->getNormal();
						if (dir1.dot(dir2) != 0)
						{
							err3 += pow(polyin[j]->getSegments()->get_length(), 3.0)*abs(dir1.cross(dir2).length() / dir1.dot(dir2)) / 3.f;
						}
					}
					if (err3 > GAP)
					{
						if (polyin[end_id - 1]->getRib()->Nodes().front() == polyin[end_id - 1])
						{
							polyin[end_id - 1]->getRib()->Nodes().erase(polyin[end_id - 1]->getRib()->Nodes().begin());
						}
						break;
					}
				}
				start_id = end_id-1;
			} while (start_id < polyin.size() - 1);
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
