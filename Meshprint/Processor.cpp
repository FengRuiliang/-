#include "Processor.h"
#include "QDebug"
Processor::Processor()
{
}


Processor::~Processor()
{
}

void Processor::read_obj_file(const char* file_name)
{
	std::ifstream cin(file_name);
	std::string line, attribute;
	double value;
	while (getline(cin, line))
	{
		std::istringstream record(line);
		record >> attribute;
		if (attribute == "v")
		{
			std::vector<double> positon;
			while (record >> value)
				positon.push_back(value);
			if (positon.size() == 3)
			{
				K::Point_3 p(positon[0], positon[1], positon[2]);
				mesh.add_vertex(p);
			}

		}
		else if (attribute == "f")
		{
			std::vector<Mesh::Vertex_index> vertex_id;
			while (record >> value)
			{
				vertex_id.push_back(Mesh::Vertex_index(value - 1));
			}
			if (vertex_id.size() == 3)
			{
				mesh.add_face(vertex_id[0], vertex_id[1], vertex_id[2]);
			}
		}
	}
}
void Processor::update_my_mesh()
{
	Mesh::Property_map<face_descriptor, K::Vector_3> fnormals = mesh.add_property_map<face_descriptor, K::Vector_3>
		("f:normals", CGAL::NULL_VECTOR).first;
	Mesh::Property_map<vertex_descriptor, K::Vector_3> vnormals = mesh.add_property_map < vertex_descriptor, K::Vector_3 >
		("v:normals", CGAL::NULL_VECTOR).first;

	CGAL::Polygon_mesh_processing::compute_normals(mesh,
		vnormals,
		fnormals,
		CGAL::Polygon_mesh_processing::parameters::vertex_point_map(mesh.points()).
		geom_traits(K()));
}
void Processor::do_slice()
{
	if (mesh.is_empty())
	{
		return;
	}
	std::vector<bool> is_unavailable(mesh.faces().size());
	const float thickness = 0.09;
	AABB_tree tree(edges(mesh).first, edges(mesh).second, mesh);
	const int num_of_slice = tree.bbox().zmax() / thickness;
	std::vector<std::vector<Mesh::Face_index>> faces_in_slices(num_of_slice);

}
void Processor::add_support()
{
	if (contours.empty())
	{
		do_slice();
	}
	Paths last_contour;
	support_region_voronoi_diagrams.clear();
	support_region_voronoi_diagrams.resize(contours.size());
	for (int slice_id = 0; slice_id < contours.size(); slice_id++)
	{
		Paths cur_contour, different_contour;
		for (auto iterP = contours[slice_id].begin(); iterP != contours[slice_id].end(); iterP++)
		{
			Path poly;
			for (auto iterV = iterP->begin(); iterV != iterP->end(); iterV++)
			{
				poly << IntPoint(iterV->x()*1e3, iterV->y()*1e3);
			}
			cur_contour << poly;
		}
		Clipper sov;
		sov.AddPaths(cur_contour, ptSubject, true);
		sov.AddPaths(last_contour, ptClip, true);
		//last_contour = cur_contour;
		sov.Execute(ctDifference, different_contour, pftNonZero, pftNonZero);
		for (auto iterD = different_contour.begin(); iterD != different_contour.end(); iterD++)
		{	
			VD vd;
			for (int id=0;id<iterD->size();id++)
			{
				Point_2 pf(iterD->at(id).X, iterD->at(id).Y);
				Point_2 pb(iterD->at((id + 1) % iterD->size()).X, iterD->at((id + 1) % iterD->size()).Y);
				Site_2 site=Site_2::construct_site_2(pf, pb);
				vd.insert(site);
			}
			VD::Edge_iterator 	eiterB = vd.edges_begin();
			VD::Edge_iterator 	eiterE = vd.edges_end();
			for (; eiterB !=eiterE; eiterB++)
			{	
				if (eiterB->is_segment())
				{
					std::tuple<double, double, double, double> seg = std::make_tuple(
						(double)eiterB->source()->point().x(), (double)eiterB->source()->point().y(),
						(double)eiterB->target()->point().x(), (double)eiterB->target()->point().y());
					support_region_voronoi_diagrams[slice_id].push_back(seg);

				}
			}
		}
	}
}
