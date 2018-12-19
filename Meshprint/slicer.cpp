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
	contours = new std::vector<std::vector<std::vector<Segment *>>>;
	contours->resize(num);
}




void Slicer::doslice()
{
	const std::vector<HE_face*>* faces = obj->get_faces_list();
	std::vector<std::vector<int>> pf(num);// prepare face
	std::vector<bool> is_need_sup(faces->size());
	slice_need_sup.resize(num,false);
	for (int i = 0; i < faces->size(); i++)
	{
		if (acos(faces->at(i)->normal().dot(Vec3f(0, 0, -1))) * 180 / 3.14 < 30)
		{
			is_need_sup[i] = true;
		}
		HE_edge* sta = faces->at(i)->pedge_;
		HE_edge* cur = sta;
		float min_z = 1e6;
		float max_z = -1e6;
		do
		{
			min_z = min_z < cur->pvert_->position().z() ? min_z : cur->pvert_->position().z();
			max_z = max_z > cur->pvert_->position().z() ? max_z : cur->pvert_->position().z();
			cur = cur->pnext_;
		} while (cur != sta);
		for (int j = 0; (float)j*thickness+0.04 - max_z<-1e-3; j++)
		{
			if ((float)j*thickness+0.04 > min_z)
			{
				pf[j].push_back(i);
			}
		}
	}
	float hei;
	for (int i=0;i<num;i++)
	{
		hei = i*0.09+0.04;
		std::vector<bool> mark(obj->num_of_face_list(), false);
		std::vector<std::vector<Segment*>> polygon;
		for (int j=0;j<pf[i].size();j++)
		{
			if (!mark[pf[i][j]]&&(!is_need_sup[pf[i][j]]||j==pf[i].size()-1))// make sure first polyline is not need support
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
						
						Vec3f p1 = ecur->pprev_->pvert_->position();
						Vec3f p2 = ecur->pvert_->position();
						if (p1.z()>hei&&p2.z()<hei)
						{
							seg->set_v1((hei - p1.z()) / (p2.z() - p1.z())*(p2 - p1) + p1);
						}
						else if (p1.z() < hei&&p2.z() > hei)
						{
							seg->set_v2((hei - p1.z()) / (p2.z() - p1.z())*(p2 - p1) + p1);
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
				if (!polyline.empty())
				{
					polygon.push_back(polyline);
				}
				
			}
		}
		contours->at(i)=polygon;
	}
	//offset();
}
void Slicer::offset()
{

	c_paths=new std::vector<Paths>(contours->size());

	ClipperOffset sov;
		for (int i = 0; i < contours->size(); i++)
		{
			Paths paths;
			for (int j=0;j<contours->at(i).size();j++)
			{
				Path path;
				for (int k=0;k<contours->at(i)[j].size();k++)
				{
					path << IntPoint(contours->at(i)[j][k]->get_v1().x()*1e3, contours->at(i)[j][k]->get_v1().y()*1e3);
				}
				paths << path;
			}
			sov.Clear();
			sov.AddPaths(paths, jtMiter, etClosedPolygon);
			Paths re;
			sov.Execute(re, -384);
			c_paths->at(i)=re;
		}
}

