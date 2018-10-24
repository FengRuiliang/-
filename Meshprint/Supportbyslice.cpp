#include "Supportbyslice.h"


Supportbyslice::Supportbyslice()
{
}


Supportbyslice::~Supportbyslice()
{
}

void Supportbyslice::construct_by_slices(SliceCut* target)
{
	std::vector < std::vector<cutLine>* >*tc = target->GetPieces();

	
	std::vector<Paths> contours;
	std::vector<std::vector<std::vector<float>>>   angle_for_slices;
	for (int i = 0; i < target->num_pieces_; i++)
	{

		Paths polygon;
		std::vector<std::vector<float>> angles_for_polygon;
		for (size_t j = 0; j < tc[i].size(); j++)
		{
			Path polyline;
			std::vector<float> angles_for_polyline;
			for (int k = 0; k < (tc[i])[j]->size(); k++)
			{
				polyline << IntPoint((tc[i])[j]->at(k).position_vert[0].x()*1e3,
					(tc[i])[j]->at(k).position_vert[0].y()*1e3);

				angles_for_polyline.push_back((tc[i])[j]->at(k).angle_belong_facet);

			}

			polygon.push_back(polyline);
			angles_for_polygon.push_back(angles_for_polyline);
		}
		contours.push_back(polygon);
		angle_for_slices.push_back(angles_for_polygon);
	}


	Clipper clipper;
	ClipperOffset offsetor;

	Paths safe_region;

	offsetor.AddPaths(contours.front(), jtMiter, etClosedPolygon);
	offsetor.Execute(safe_region,3.0);

	for (int i=1;i<contours.size();i++)
	{
		Paths overhang;
		clipper.AddPaths(safe_region, ptClip, true);
		clipper.AddPaths(contours[i], ptSubject, true);
		clipper.Execute(ctDifference, overhang, pftNonZero, pftNonZero);
		if (!overhang.empty())
		{
			sample_support_point(tc[i-1], tc[i]);
			offsetor.Clear();
			offsetor.AddPaths(contours[i], jtMiter, etClosedPolygon);
			offsetor.Execute(safe_region, 3.0);
		}
		else
		{
			clipper.Clear();
			clipper.AddPaths(safe_region, ptClip, true);
			clipper.AddPaths(contours[i], ptSubject, true);
			clipper.Execute(ctIntersection, safe_region, pftNonZero,pftNonZero);

			offsetor.Clear();
			offsetor.AddPaths(safe_region, jtMiter, etClosedPolygon);
			offsetor.Execute(safe_region, 3.0);
		}
	}
}

void Supportbyslice::sample_support_point(std::vector < std::vector<cutLine>* > safe_region, 
	std::vector < std::vector<cutLine>* > contours)
{
	for (int i=0;i<safe_region.size();i++)
	{
		for (int j=0;j<safe_region[i]->size();j++)
		{
		
		}
	}
	
	
}
