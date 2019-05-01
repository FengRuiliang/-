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
	candidate_nodes_.resize(cnts->size());
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
						std::vector<Node*> added;
						sup_paths[i].push_back(added);
						Node * node_ = new Node(var[j]);

						sup_paths[i].back().push_back(node_);
						candidate_nodes_[i].push_back(node_);

					}
					else if (count1 % 2 == 0 && count2 % 2 == 0)
					{
						Node * node_ = new Node(var[j]);
						if (j==0)
						{
							std::vector<Node*> added;
							sup_paths[i].push_back(added);
						}
						sup_paths[i].back().push_back(node_);
						candidate_nodes_[i].push_back(node_);
					}
					else if (count1 % 2 == 0 && count2 % 2 != 0)
					{
						Node * node_ = new Node(var[j]);
						sup_paths[i].back().push_back(node_);
					}	
				}
			}
			link_to_ribs(candidate_nodes_[i]);
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
		if (nodes[j]->getRibs().empty())
		{
			Rib* rib_=new Rib(nodes[j]);
			ribs.push_back(rib_);
			nodes[j]->setRib(rib_);
		}
	}

}
void Supportor::opitimizePointsandRibs()
{
	for (int i = 298; i < sup_paths.size(); i++)
	{
		for (int j=0;j<sup_paths[i].size();j++)
		{
			qDebug() << i;
			std::vector<Node*>& polyin = sup_paths[i][j];
			Node* sta = polyin.front();
			for (int k=0;k<polyin.size();k++)
			{
				polyin[k]->setNext(polyin[(k + 1) % polyin.size()]);
				if (!polyin[k]->isHeadforALlRibs())
				{
					sta = polyin[k];
					break;
				}
			}
			Node* las = sta;
			for (Node* end=sta->getNext();end!=sta;end=end->getNext())
			{
				if (!end->isHeadforALlRibs())
				{
					float err3 = 0;
					for (Node* cur = las; cur != end; cur = cur->getNext())
					{
						Vec3f dir1 = cur->getPosition() - las->getSegments()->get_v1();
						dir1.normalize();
						Vec3f dir2 = cur->Normal();
						err3 += pow(cur->getSegments()->get_length(), 3.0)*abs(dir1.cross(dir2).length() / dir1.dot(dir2)) / 3.f;
						for (int k = 0; k < cur->getRibs().size(); k++)
						{
							cur->getRibs()[k]->popFront();
						}
						cur->getRibs().clear();
					}
				}		
			}
		}
	}
}
bool Node::isHeadforALlRibs()
{
	bool is_head_for_all_rib = true;
	for (int r = 0; r < ribs.size(); r++)
	{
		Node* cur = ribs[r]->getHeadNodePtr();
		is_head_for_all_rib == is_head_for_all_rib &&  this== cur;
	}
	return is_head_for_all_rib;
}