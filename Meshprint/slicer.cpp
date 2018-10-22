#include "slicer.h"



slicer::slicer()
{
}


slicer::~slicer()
{
}

slicer::slicer(Mesh3D * tar, float in_thickness)
{
	obj = tar;
	thickness = in_thickness;
	num=(int)(obj->getBoundingBox()[0][2]/thickness)+1;
}


void slicer::execute()
{
	const std::vector<HE_face*>* faces = obj->get_faces_list();
	std::vector<std::vector<int>> prepross_faces(num);
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
			prepross_faces[j].push_back(i);
		}
	}
	for (int i=1;i<num;i++)
	{
		prepross_faces[i];
	}



}
