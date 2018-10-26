#include "slicer.h"



Slicer::Slicer()
{
}


Slicer::~Slicer()
{
}

Slicer::Slicer(Mesh3D * tar, float in_thickness)
{
	obj = tar;
	thickness = in_thickness;
	num=(int)(obj->getBoundingBox()[0][2]/thickness)+1;
	contours.resize(num);
}


void Slicer::execute()
{
	const std::vector<HE_face*>* faces = obj->get_faces_list();
	std::vector<std::vector<int>> pf(num);// prepare face
	for (int i = 0; i < faces->size(); i++)
	{
		HE_edge* sta = faces->at(i)->pedge_;
		HE_edge* cur = sta;
		float min_z = 1e6;
		float max_z = -1e6;
		do 
		{
			min_z = min_z < cur->pvert_->position().z() ? min_z : cur->pvert_->position().z();
			max_z = max_z > cur->pvert_->position().z() ? max_z : cur->pvert_->position().z();
		} while (cur!=sta);
		for (int j = min_z / thickness; j <= max_z / thickness; j++)
		{
			pf[j].push_back(i);
		}
	}
	
	for (int i=1;i<num;i++)
	{
		std::vector<bool> mark(obj->num_of_face_list(), false);
		std::vector<std::vector<Segment*>> polygon;
		for (int j=0;j<pf[i].size();j++)
		{
			if (!mark[pf[i][j]]);
			{
				std::vector<Segment*> polyline;
				HE_edge *ejump = faces->at(pf[i][j])->pedge_;
				do
				{
					mark[ejump->pface_->id()] = true;
					Segment *seg=new Segment;
					HE_edge *esta = ejump;
					HE_edge *ecur = ejump;
					do
					{
						Vec3f p1 = ecur->pvert_->position();
						Vec3f p2 = ecur->pprev_->pvert_->position();
						if (p1.z()>i*thickness&&p2.z()<i*thickness)
						{
							seg->set_v1((i*thickness - p1.z()) / (p2.z() - p1.z())*(p2 - p1) + p1);
						}
						else if (p1.z() < i*thickness&&p2.z() > i*thickness)
						{
							seg->set_v2((i*thickness - p1.z()) / (p2.z() - p1.z())*(p2 - p1) + p1);
							ejump = ecur->ppair_;
						}
						ecur = ecur->pnext_;
					} while (ecur != esta);
					//判断线段是否合理，如果合理设置角度，并添加到环中。
					if (seg->vailed())
					{
						seg->set_angle(acos(ecur->pface_->normal()*Vec3f(0, 0, -1)) * 180 / 3.14);
						polyline.push_back(seg);
					}
					else
					{
						delete seg;
					}
					
				} while (!mark[ejump->pface_->id()]);
				polygon.push_back(polyline);
			}
		}
		contours[i]=polygon;
	}
}
