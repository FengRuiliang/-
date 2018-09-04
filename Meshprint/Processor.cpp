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
	// Use the Slicer constructor from a pre-built AABB_tree
	AABB_tree tree(edges(mesh).first, edges(mesh).second, mesh);
	CGAL::Polygon_mesh_slicer<Mesh, K> slicer_aabb(mesh, tree);
	float z_height = 0, zmax = tree.bbox().zmax();
	while (z_height < tree.bbox().zmax())
	{
		Polylines current_polylines;
		slicer_aabb(K::Plane_3(0, 0, 1, -z_height), std::back_inserter(current_polylines));
		contours.push_back(current_polylines);
		z_height += 1.0;
	}
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
		last_contour = cur_contour;
		sov.Execute(ctDifference, different_contour, pftNonZero, pftNonZero);
		SDG2          sdg;
		SDG2::Site_2  site;
		for (auto iterD = different_contour.begin(); iterD != different_contour.end(); iterD++)
		{
			//polygon points
			std::vector<Gt::Point_2> points;
			//segments of the polygon as a pair of point indices
			std::vector<std::pair<std::size_t, std::size_t> > indices;
			SDG2::Site_2 site;
			//read a close polygon given by its segments
			// s x0 y0 x1 y1
			// s x1 y1 x2 y2
			// ...
			// s xn yn x0 y0
			ifs >> site;
			assert(site.is_segment());
			points.push_back(site.source_of_supporting_site());
			std::size_t k = 0;
			while (ifs >> site) {
				assert(site.is_segment());
				points.push_back(site.source_of_supporting_site());
				indices.push_back(std::make_pair(k, k + 1));
				++k;
			}
			indices.push_back(std::make_pair(k, 0));
			ifs.close();
			SDG2          sdg;
			//insert the polygon segments all at once using spatial sorting to speed the insertion
			sdg.insert_segments(points.begin(), points.end(), indices.begin(), indices.end());
			// validate the segment Delaunay graph
			assert(sdg.is_valid(true, 1));
			return 0;
		}
	}
}
